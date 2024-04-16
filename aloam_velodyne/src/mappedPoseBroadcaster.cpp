#include <math.h>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

//#include "lidarFactor.hpp"
//#include "aloam_velodyne/common.h"

// Comes from mapping
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d t_wmap_wodom(0, 0, 0);

// Comes from odometry
nav_msgs::msg::Odometry laserOdometry;

std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pubOdomAftMappedHighFrec;
std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;
//std::shared_ptr<rclcpp::Node> node;

void computeAndPublish()
{
	// high frequence publish
	Eigen::Vector3d t_wodom_curr;
	Eigen::Quaterniond q_wodom_curr;

	t_wodom_curr.x() = laserOdometry.pose.pose.position.x;
	t_wodom_curr.y() = laserOdometry.pose.pose.position.y;
	t_wodom_curr.z() = laserOdometry.pose.pose.position.z;
	q_wodom_curr.x() = laserOdometry.pose.pose.orientation.x;
	q_wodom_curr.y() = laserOdometry.pose.pose.orientation.y;
	q_wodom_curr.z() = laserOdometry.pose.pose.orientation.z;
	q_wodom_curr.w() = laserOdometry.pose.pose.orientation.w;

	Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
	Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom; 

	nav_msgs::msg::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "map";
	odomAftMapped.child_frame_id = "base_link";
	odomAftMapped.header.stamp = laserOdometry.header.stamp;
	odomAftMapped.pose.pose.position.x = t_w_curr.x();
	odomAftMapped.pose.pose.position.y = t_w_curr.y();
	odomAftMapped.pose.pose.position.z = t_w_curr.z();
	odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
	odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
	odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
	odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
	pubOdomAftMappedHighFrec->publish(odomAftMapped);

	geometry_msgs::msg::TransformStamped transformStamped;
	transformStamped.header = odomAftMapped.header;
	transformStamped.child_frame_id = odomAftMapped.child_frame_id;
	transformStamped.transform.translation.x = t_w_curr.x(); 
	transformStamped.transform.translation.y = t_w_curr.y();
	transformStamped.transform.translation.z = t_w_curr.z();
	transformStamped.transform.rotation = odomAftMapped.pose.pose.orientation;
	broadcaster->sendTransform(transformStamped);
}

//receive odometry
void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odom)
{
	// odometryBuf.push(laserOdometry);
	laserOdometry = *odom;
	computeAndPublish();
}

void poseHandler(const geometry_msgs::msg::Pose::SharedPtr pose){
	t_wmap_wodom.x() = pose->position.x;
	t_wmap_wodom.y() = pose->position.y;
	t_wmap_wodom.z() = pose->position.z;
	q_wmap_wodom.x() = pose->orientation.x;
	q_wmap_wodom.y() = pose->orientation.y;
	q_wmap_wodom.z() = pose->orientation.z;
	q_wmap_wodom.w() = pose->orientation.w;
	computeAndPublish();
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("mappedPoseBroadcaster");

	auto subLaserOdometry = node->create_subscription<nav_msgs::msg::Odometry>("/laser_odom_to_init", rclcpp::SensorDataQoS(), odometryHandler);
	auto subMappingPose = node->create_subscription<geometry_msgs::msg::Pose>("/odom_mapping_pose", rclcpp::SensorDataQoS(), poseHandler);
	broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
	pubOdomAftMappedHighFrec = node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init_high_frec", 10);
	
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();

	return 0;
}
