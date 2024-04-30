// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <memory>
#include <string>

#include <cuda.h>
#include <cuda_runtime.h>
#include <nvToolsExt.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lidar_slam_msgs/msg/lidar_slam.hpp>
#include "time_profiling_spinner/time_profiling_spinner.h"

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "aloam_velodyne/cudaMapping.h"
#include "lidarFactor.hpp"

// Make it false to save some execution time
bool pubClouds = false;

int laserCloudCenWidth = 10;
int laserCloudCenHeight = 10;
int laserCloudCenDepth = 5;
const int laserCloudWidth = 21;
const int laserCloudHeight = 21;
const int laserCloudDepth = 11;
const double MAGICNUM = 50.0;
const double MAGICNUM_HALF = MAGICNUM/2;

const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851

int laserCloudValidInd[125];
int laserCloudSurroundInd[125];

// input: from odom
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());

// ouput: all visualble cube points
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());

// surround points in map to build tree
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());

//input & output: points in one frame. local --> global
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

// points in every cube
pcl::PointCloud<PointType>::Ptr *laserCloudCornerArray = new pcl::PointCloud<PointType>::Ptr[laserCloudNum];
pcl::PointCloud<PointType>::Ptr *laserCloudSurfArray = new pcl::PointCloud<PointType>::Ptr[laserCloudNum];

double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);

// transformation between odom's world and map's world frame
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d t_wmap_wodom(0, 0, 0);

Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d t_wodom_curr(0, 0, 0);

pcl::VoxelGrid<PointType> downSizeFilterCorner;
pcl::VoxelGrid<PointType> downSizeFilterSurf;

PointType pointSel;

std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubLaserCloudSurround, 
    pubLaserCloudMap, pubLaserCloudFullRes;
std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Pose>> pubOdomMappingPose;

std::shared_ptr<rclcpp::Node> node;

cudaStream_t cornerStream, surfStream;

// set initial guess
void transformAssociateToMap()
{
    q_w_curr = q_wmap_wodom * q_wodom_curr;
    t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}

void transformUpdate()
{
    q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
    t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}

void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}

// void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
// {
//     Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
//     Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
//     po->x = point_curr.x();
//     po->y = point_curr.y();
//     po->z = point_curr.z();
//     po->intensity = pi->intensity;
// }

void pcToFloat3Arr(const pcl::PointCloud<PointType>::Ptr point_cloud_ptr, float3* arr){
    auto& points = point_cloud_ptr->points;
    auto numPoints = points.size();
    for(auto i=0; i< numPoints; ++i){
        arr[i].x = points[i].x;
        arr[i].y = points[i].y;
        arr[i].z = points[i].z;
    }
}

void pointAssociateToMapAll(const pcl::PointCloud<PointType>::Ptr stackPC, pcl::PointCloud<PointType>::Ptr stackPCMapped){
    auto& stackPoints = stackPC->points;
    auto& stackMappedPoints = stackPCMapped->points;
    auto numStackPoints = stackPoints.size();
    for(auto i=0; i< numStackPoints; ++i){
        // doing pointAssociateToMap
        Eigen::Vector3d point_curr(stackPoints[i].x, stackPoints[i].y, stackPoints[i].z);
        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
        stackMappedPoints[i].x = point_w.x();
        stackMappedPoints[i].y = point_w.y();
        stackMappedPoints[i].z = point_w.z();
    }
}

void pointAssociateToMapAll2(const pcl::PointCloud<PointType>::Ptr stackPC, float3* stackPCMapped){
    auto& stackPoints = stackPC->points;
    auto numStackPoints = stackPoints.size();
    for(auto i=0; i< numStackPoints; ++i){
        // doing pointAssociateToMap
        Eigen::Vector3d point_curr(stackPoints[i].x, stackPoints[i].y, stackPoints[i].z);
        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
        stackPCMapped[i].x = point_w.x();
        stackPCMapped[i].y = point_w.y();
        stackPCMapped[i].z = point_w.z();
    }
}



void lidarSLAMHandler(const lidar_slam_msgs::msg::LidarSLAM::SharedPtr lidarSLAMPtr)
{
    TicToc t_whole;

    laserCloudCornerLast->clear();
    laserCloudSurfLast->clear();
    laserCloudFullRes->clear();
    pcl::fromROSMsg(lidarSLAMPtr->point_clouds[0], *laserCloudCornerLast);
    pcl::fromROSMsg(lidarSLAMPtr->point_clouds[1], *laserCloudSurfLast);
    pcl::fromROSMsg(lidarSLAMPtr->point_clouds[2], *laserCloudFullRes);
    auto odom = lidarSLAMPtr->odom;
    q_wodom_curr.x() = odom.pose.pose.orientation.x;
    q_wodom_curr.y() = odom.pose.pose.orientation.y;
    q_wodom_curr.z() = odom.pose.pose.orientation.z;
    q_wodom_curr.w() = odom.pose.pose.orientation.w;
    t_wodom_curr.x() = odom.pose.pose.position.x;
    t_wodom_curr.y() = odom.pose.pose.position.y;
    t_wodom_curr.z() = odom.pose.pose.position.z;

    transformAssociateToMap();

    int centerCubeI = int((t_w_curr.x() + MAGICNUM_HALF) / MAGICNUM) + laserCloudCenWidth;
    int centerCubeJ = int((t_w_curr.y() + MAGICNUM_HALF) / MAGICNUM) + laserCloudCenHeight;
    int centerCubeK = int((t_w_curr.z() + MAGICNUM_HALF) / MAGICNUM) + laserCloudCenDepth;

    if (t_w_curr.x() + MAGICNUM_HALF < 0)
        centerCubeI--;
    if (t_w_curr.y() + MAGICNUM_HALF < 0)
        centerCubeJ--;
    if (t_w_curr.z() + MAGICNUM_HALF < 0)
        centerCubeK--;

    while (centerCubeI < 3)
    {
        for (int j = 0; j < laserCloudHeight; j++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            { 
                int i = laserCloudWidth - 1;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]; 
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; i >= 1; i--)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeI++;
        laserCloudCenWidth++;
    }

    while (centerCubeI >= laserCloudWidth - 3)
    { 
        for (int j = 0; j < laserCloudHeight; j++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int i = 0;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; i < laserCloudWidth - 1; i++)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeI--;
        laserCloudCenWidth--;
    }

    while (centerCubeJ < 3)
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int j = laserCloudHeight - 1;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; j >= 1; j--)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ++;
        laserCloudCenHeight++;
    }

    while (centerCubeJ >= laserCloudHeight - 3)
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int j = 0;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; j < laserCloudHeight - 1; j++)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ--;
        laserCloudCenHeight--;
    }

    while (centerCubeK < 3)
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int j = 0; j < laserCloudHeight; j++)
            {
                int k = laserCloudDepth - 1;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; k >= 1; k--)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK++;
        laserCloudCenDepth++;
    }

    while (centerCubeK >= laserCloudDepth - 3)
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int j = 0; j < laserCloudHeight; j++)
            {
                int k = 0;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                for (; k < laserCloudDepth - 1; k++)
                {
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                }
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK--;
        laserCloudCenDepth--;
    }
    //RCLCPP_WARN(node->get_logger(), "mapping shift time: %f ms",
    //        t_shift.toc());

    //TicToc t_assign;
    int laserCloudValidNum = 0;
    int laserCloudSurroundNum = 0;

    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
    {
        for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
        {
            for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
            {
                if (i >= 0 && i < laserCloudWidth &&
                        j >= 0 && j < laserCloudHeight &&
                        k >= 0 && k < laserCloudDepth &&
                        laserCloudValidNum < 125)
                { 
                    laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                    laserCloudValidNum++;
                    laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                    laserCloudSurroundNum++;
                }
            }
        }
    }

    pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerStack);
    int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

    pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfStack);
    int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();

    pcl::PointCloud<PointType>::Ptr laserCloudCornerStackMapped(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr laserCloudSurfStackMapped(new pcl::PointCloud<PointType>());
    laserCloudCornerStackMapped->width = laserCloudCornerStack->width;
    laserCloudCornerStackMapped->height = laserCloudCornerStack->height;
    laserCloudCornerStackMapped->points.resize(laserCloudCornerStackMapped->width * laserCloudCornerStackMapped->height);
    laserCloudSurfStackMapped->width = laserCloudSurfStack->width;
    laserCloudSurfStackMapped->height = laserCloudSurfStack->height;
    laserCloudSurfStackMapped->points.resize(laserCloudSurfStackMapped->width * laserCloudSurfStackMapped->height);

    pointAssociateToMapAll(laserCloudCornerStack, laserCloudCornerStackMapped);
    pointAssociateToMapAll(laserCloudSurfStack, laserCloudSurfStackMapped);

    PointType minpCorner, maxpCorner, minpSurf, maxpSurf;
    pcl::getMinMax3D(*laserCloudCornerStackMapped, minpCorner, maxpCorner);
    pcl::getMinMax3D(*laserCloudSurfStackMapped, minpSurf, maxpSurf);
    const float margin = 2.0; // Increasing this margin might make mapping more accurate at the cost of execution time
    const float cornerminx = minpCorner.x - margin, cornerminy = minpCorner.y - margin, cornerminz = minpCorner.z - margin;
    const float cornermaxx = maxpCorner.x + margin, cornermaxy = maxpCorner.y + margin, cornermaxz = maxpCorner.z + margin;
    const float surfminx = minpSurf.x - margin, surfminy = minpSurf.y - margin, surfminz = minpSurf.z - margin;
    const float surfmaxx = maxpSurf.x + margin, surfmaxy = maxpSurf.y + margin, surfmaxz = maxpSurf.z + margin;
    for (int i = 0; i < laserCloudValidNum; i++)
    {
        pcl::PointCloud<PointType>::Ptr cornerCloud = laserCloudCornerArray[laserCloudValidInd[i]];
        for (auto& p : cornerCloud->points){
            if(p.x > cornerminx && p.x < cornermaxx && p.y > cornerminy && p.y < cornermaxy &&
                    p.z > cornerminz && p.z < cornermaxz){
                laserCloudCornerFromMap->push_back(p);
            }
        }
        pcl::PointCloud<PointType>::Ptr surfCloud = laserCloudSurfArray[laserCloudValidInd[i]];
        for (auto& p : surfCloud->points){
            if(p.x > surfminx && p.x < surfmaxx && p.y > surfminy && p.y < surfmaxy &&
                    p.z > surfminz && p.z < surfmaxz){
                laserCloudSurfFromMap->push_back(p);
            }
        }
    }
    int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
    int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

    // Don't build the trees for the entire map points, rather focus on the part of the map that is close to the
    // stack point clouds
    TicToc t_solver; 
    if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
    {
        int *surfOutPointIndexes=0;
        float3 *cornerRef=0, *cornerQuery=0, *surfRef=0, *surfQuery=0; // *d_surfRef=0, 
        Eigen::Vector3d *cornerOutPoints=0; // 3 output points per query

        cudaMallocManaged((char **)&cornerRef, laserCloudCornerFromMapNum*sizeof(float3));
        cudaMallocManaged((char **)&surfRef, laserCloudSurfFromMapNum*sizeof(float3));
        cudaMallocManaged((char **)&cornerQuery, laserCloudCornerStackNum*sizeof(float3));
        cudaMallocManaged((char **)&surfQuery, laserCloudSurfStackNum*sizeof(float3));
        cudaMallocManaged((char **)&cornerOutPoints, laserCloudCornerStackNum*sizeof(Eigen::Vector3d)*2);
        cudaMallocManaged((char **)&surfOutPointIndexes, laserCloudSurfStackNum*sizeof(int)*NUM_NEIGHBORS);

        if (!cornerRef || !cornerQuery || !cornerOutPoints || !surfRef || !surfQuery || !surfOutPointIndexes){
            RCLCPP_ERROR(node->get_logger(), "Could not allocate points mem, cudaMallocManaged or cudaMalloc failed.");
            return;
        }

        // These will be nodes of the trees
        pcToFloat3Arr(laserCloudSurfFromMap, surfRef);
        pcToFloat3Arr(laserCloudCornerFromMap, cornerRef);

        nvtxRangePush("Build_kdtree");
        buildKDTree(cornerRef, laserCloudCornerFromMapNum);
        buildKDTree(surfRef, laserCloudSurfFromMapNum);
        cudaDeviceSynchronize();
        nvtxRangePop();

        // Is it ok? especially for ref?
        cudaStreamAttachMemAsync(cornerStream, cornerRef);
        cudaStreamAttachMemAsync(cornerStream, cornerQuery);
        cudaStreamAttachMemAsync(cornerStream, cornerOutPoints);
        cudaStreamAttachMemAsync(surfStream, surfRef);
        cudaStreamAttachMemAsync(surfStream, surfQuery);
        cudaStreamAttachMemAsync(surfStream, surfOutPointIndexes);

        for (int iterCount = 0; iterCount < 2; iterCount++)
        {
            // TicToc t_solver2;
            nvtxRangePush("Knn_kernels");
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Manifold *q_parameterization = new ceres::EigenQuaternionManifold();
            ceres::Problem::Options problem_options;

            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(parameters, 4, q_parameterization);
            problem.AddParameterBlock(parameters + 4, 3);

            // Need to do mapping here as well, this is already superfast on cpu
            pointAssociateToMapAll2(laserCloudCornerStack, cornerQuery);
            pointAssociateToMapAll2(laserCloudSurfStack, surfQuery);

            run_kernel_surf(surfOutPointIndexes, surfQuery, laserCloudSurfStackNum, 
                    surfRef, laserCloudSurfFromMapNum, 2.0f, surfStream);
            run_kernel_corner(cornerOutPoints, cornerQuery, laserCloudCornerStackNum, 
                    cornerRef, laserCloudCornerFromMapNum, 2.0f, cornerStream);
            cudaDeviceSynchronize();
            auto& stackPoints = laserCloudCornerStack->points;
            for (int i = 0; i < laserCloudCornerStackNum; i++)
            {
                if (!cornerOutPoints[i*2].isZero()){
                    Eigen::Vector3d curr_point(stackPoints[i].x, stackPoints[i].y, stackPoints[i].z);
                    ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, 
                            cornerOutPoints[i*2], cornerOutPoints[i*2+1], 1.0);
                    problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                }
            }

            // auto& mapPoints = laserCloudSurfFromMap->points;
            Eigen::Matrix<double, NUM_NEIGHBORS, 3> matA0;
            for (int i = 0; i < laserCloudSurfStackNum; i++)
            {
                auto idx = i*NUM_NEIGHBORS;
                if (surfOutPointIndexes[idx] != -1){
                    for (int j = 0; j < NUM_NEIGHBORS; j++)
                    {
                        auto idx2 = surfOutPointIndexes[idx+j];
                        matA0(j, 0) = surfRef[idx2].x;
                        matA0(j, 1) = surfRef[idx2].y;
                        matA0(j, 2) = surfRef[idx2].z;
                    }

                    // find the norm of plane
                    // I can't put the following code in cuda kernel, so I do it in cpu
                    Eigen::Matrix<double, NUM_NEIGHBORS, 1> matB0 = -1 * 
                        Eigen::Matrix<double, NUM_NEIGHBORS, 1>::Ones();
                    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                    double negative_OA_dot_norm = 1 / norm.norm();
                    norm.normalize();

                    // Here n(pa, pb, pc) is unit norm of plane
                    bool planeValid = true;
                    for (int j = 0; j < NUM_NEIGHBORS; j++)
                    {
                        auto idx2 = surfOutPointIndexes[idx+j];
                        // if OX * n > 0.2, then plane is not fit well
                        if (fabs(norm(0) * surfRef[idx2].x +
                                    norm(1) * surfRef[idx2].y +
                                    norm(2) * surfRef[idx2].z + negative_OA_dot_norm) > 0.2)
                        {
                            planeValid = false;
                            break;
                        }
                    }

                    if (planeValid)
                    {
                        Eigen::Vector3d curr_point(laserCloudSurfStack->points[i].x, laserCloudSurfStack->points[i].y, 
                                laserCloudSurfStack->points[i].z);
                        ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                        problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                    }
                }
            }

            // RCLCPP_WARN(node->get_logger(), "mapping solver part 2 time: %f ms\n", t_solver2.toc());
            nvtxRangePop();
            nvtxRangePush("Ceres");
            // TicToc t_solver3;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            //options.dense_linear_algebra_library_type = ceres::CUDA; // CUDA is underutilizing the GPU
            options.dense_linear_algebra_library_type = ceres::EIGEN;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            nvtxRangePop();
            // RCLCPP_WARN(node->get_logger(), "mapping solver part 3 time: %f ms",
            //         t_solver3.toc());
        }
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "WARNING! time map corner and surf num are not enough");
    }

    //RCLCPP_WARN(node->get_logger(), "mapping solver all time: %f ms",
    //        t_solver.toc());

    TicToc t_final;
    transformUpdate();
    geometry_msgs::msg::Pose pose;
    pose.position.x = t_wmap_wodom.x();
    pose.position.y = t_wmap_wodom.y();
    pose.position.z = t_wmap_wodom.z();
    pose.orientation.x = q_wmap_wodom.x();
    pose.orientation.y = q_wmap_wodom.y();
    pose.orientation.z = q_wmap_wodom.z();
    pose.orientation.w = q_wmap_wodom.w();
    pubOdomMappingPose->publish(pose);

    for (int i = 0; i < laserCloudCornerStackNum; i++)
    {
        pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

        int cubeI = int((pointSel.x + MAGICNUM_HALF) / MAGICNUM) + laserCloudCenWidth;
        int cubeJ = int((pointSel.y + MAGICNUM_HALF) / MAGICNUM) + laserCloudCenHeight;
        int cubeK = int((pointSel.z + MAGICNUM_HALF) / MAGICNUM) + laserCloudCenDepth;

        if (pointSel.x + MAGICNUM_HALF < 0)
            cubeI--;
        if (pointSel.y + MAGICNUM_HALF < 0)
            cubeJ--;
        if (pointSel.z + MAGICNUM_HALF < 0)
            cubeK--;

        if (cubeI >= 0 && cubeI < laserCloudWidth &&
                cubeJ >= 0 && cubeJ < laserCloudHeight &&
                cubeK >= 0 && cubeK < laserCloudDepth)
        {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            laserCloudCornerArray[cubeInd]->push_back(pointSel);
        }
    }

    for (int i = 0; i < laserCloudSurfStackNum; i++)
    {
        pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

        int cubeI = int((pointSel.x + MAGICNUM_HALF) / MAGICNUM) + laserCloudCenWidth;
        int cubeJ = int((pointSel.y + MAGICNUM_HALF) / MAGICNUM) + laserCloudCenHeight;
        int cubeK = int((pointSel.z + MAGICNUM_HALF) / MAGICNUM) + laserCloudCenDepth;

        if (pointSel.x + MAGICNUM_HALF < 0)
            cubeI--;
        if (pointSel.y + MAGICNUM_HALF < 0)
            cubeJ--;
        if (pointSel.z + MAGICNUM_HALF < 0)
            cubeK--;

        if (cubeI >= 0 && cubeI < laserCloudWidth &&
                cubeJ >= 0 && cubeJ < laserCloudHeight &&
                cubeK >= 0 && cubeK < laserCloudDepth)
        {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            laserCloudSurfArray[cubeInd]->push_back(pointSel);
        }
    }


    for (int i = 0; i < laserCloudValidNum; i++)
    {
        int ind = laserCloudValidInd[i];

        pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
        downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
        downSizeFilterCorner.filter(*tmpCorner);
        laserCloudCornerArray[ind] = tmpCorner;

        pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
        downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
        downSizeFilterSurf.filter(*tmpSurf);
        laserCloudSurfArray[ind] = tmpSurf;
    }
    //RCLCPP_WARN(node->get_logger(), "mapping final ops time: %f ms",
    //        t_final.toc());

    //publish surround map for every 5 frame
    if (pubClouds){
        static int frameCount = 0;
        if (frameCount % 5 == 0)
        {
            laserCloudSurround->clear();
            for (int i = 0; i < laserCloudSurroundNum; i++)
            {
                int ind = laserCloudSurroundInd[i];
                *laserCloudSurround += *laserCloudCornerArray[ind];
                *laserCloudSurround += *laserCloudSurfArray[ind];
            }

            sensor_msgs::msg::PointCloud2 laserCloudSurround3;
            pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
            laserCloudSurround3.header.stamp = odom.header.stamp;
            laserCloudSurround3.header.frame_id = "map";
            pubLaserCloudSurround->publish(laserCloudSurround3);
        }

        if (frameCount % 20 == 0)
        {
            pcl::PointCloud<PointType> laserCloudMap;
            for (int i = 0; i < 4851; i++)
            {
                laserCloudMap += *laserCloudCornerArray[i];
                laserCloudMap += *laserCloudSurfArray[i];
            }
            sensor_msgs::msg::PointCloud2 laserCloudMsg;
            pcl::toROSMsg(laserCloudMap, laserCloudMsg);
            laserCloudMsg.header.stamp = odom.header.stamp;
            laserCloudMsg.header.frame_id = "map";
            pubLaserCloudMap->publish(laserCloudMsg);
        }

        int laserCloudFullResNum = laserCloudFullRes->points.size();
        for (int i = 0; i < laserCloudFullResNum; i++)
        {
            pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = odom.header.stamp;
        laserCloudFullRes3.header.frame_id = "map";
        pubLaserCloudFullRes->publish(laserCloudFullRes3);

        frameCount++;
    }

    auto t = t_whole.toc();
    if(t > 500.0)
        RCLCPP_WARN(node->get_logger(), "MAPPING TIME: %f ms", t);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("laserMapping");

    float lineRes = 0;
    float planeRes = 0;
    node->declare_parameter<float>("mapping_line_resolution", 0.4);
    node->get_parameter<float>("mapping_line_resolution", lineRes);
    node->declare_parameter<float>("mapping_plane_resolution", 0.8);
    node->get_parameter<float>("mapping_plane_resolution", planeRes);
    node->declare_parameter<bool>("publish_clouds", false);
    node->get_parameter<bool>("publish_clouds", pubClouds);

    downSizeFilterCorner.setLeafSize(lineRes, lineRes,lineRes);
    downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

    auto sublidarSLAM = node->create_subscription<lidar_slam_msgs::msg::LidarSLAM>("/lidar_slam", rclcpp::SensorDataQoS(), lidarSLAMHandler);
    pubOdomMappingPose = node->create_publisher<geometry_msgs::msg::Pose>("/odom_mapping_pose", 10);
    if(pubClouds){
        pubLaserCloudSurround = node->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surround", 10); // optional
        pubLaserCloudMap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_map", 10); // optional
        pubLaserCloudFullRes = node->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_registered", 10); // optional
    }

    for (int i = 0; i < laserCloudNum; i++)
    {
        laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
        laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
    }

    init_cuda();
    cudaStreamCreate(&cornerStream);
    cudaStreamCreate(&surfStream);

    TimeProfilingSpinner spinner(node); //mapping 
    spinner.spinAndProfileUntilShutdown();
    rclcpp::shutdown();

    delete[] laserCloudCornerArray;
    delete[] laserCloudSurfArray;
    cudaStreamDestroy(cornerStream);
    cudaStreamDestroy(surfStream);

    return 0;
}

