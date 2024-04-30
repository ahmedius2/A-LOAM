#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <cuda.h>
#include <cuda_runtime.h>
#include "cukd/builder.h"
#include "cukd/knn.h"
#include "aloam_velodyne/cudaMapping.h"

bool init_cuda()
{
    cudaError_t err0;

    // Check that we have at least one CUDA device 
    int nb_devices;
    err0 = cudaGetDeviceCount(&nb_devices);
    if (err0 != cudaSuccess || nb_devices == 0) {
        printf("ERROR: No CUDA device found\n");
        return false;
    }

    // Select the first CUDA device as default
    err0 = cudaSetDevice(0);
    if (err0 != cudaSuccess) {
        printf("ERROR: Cannot set the chosen CUDA device\n");
        return false;
    }

    return true;
}

void buildKDTree(float3 *refPoints, int numPoints){
    cukd::buildTree_bitonic<float3>(refPoints, numPoints);
    // cukd::buildTree_inPlace<float3>(refPoints, numPoints);
    //cukd::buildTree_host<float3>(refPoints, numPoints);
}

template<typename CandidateList>
__global__
void d_knn_corner(Eigen::Vector3d* __restrict__ outPoints,
                const float3* __restrict__ d_queries,
                const int     numQueries,
                const float3* __restrict__ d_nodes,
                const int     numNodes,
                const float   cutOffRadius)
{
    int tid = threadIdx.x+blockIdx.x*blockDim.x;
    if (tid >= numQueries) return;
    // outPoints[tid*3] = Eigen::Vector3d::Zero();
    outPoints[tid*2] = Eigen::Vector3d::Zero();

    CandidateList result(cutOffRadius);
    //traversal methods: stackBased, stackFree, cct
    float sqrDist = cukd::stackBased::knn<CandidateList, float3, cukd::default_data_traits<float3>>(result,
            d_queries[tid], d_nodes, numNodes);

    if (sqrDist < 1.0)
    { 
        Eigen::Vector3d nearCorners[NUM_NEIGHBORS], center(0, 0, 0);

        for (int j = 0; j < NUM_NEIGHBORS; j++)
        {
            auto idx = result.get_pointID(j);
            nearCorners[j].x() = d_nodes[idx].x;
            nearCorners[j].y() = d_nodes[idx].y;
            nearCorners[j].z() = d_nodes[idx].z;
            center = center + nearCorners[j];
        }
        center = center / (double)NUM_NEIGHBORS;

        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
        for (int j = 0; j < NUM_NEIGHBORS; j++)
        {
            Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
            covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

        // if is indeed line feature
        // note Eigen library sort eigenvalues in increasing order
        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
        { 
            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            outPoints[tid*2] = 0.1 * unit_direction + center; // point_a
            outPoints[tid*2+1] = -0.1 * unit_direction + center; // point_b
        }		
    }
}



template<typename CandidateList>
__global__
void d_knn_surf(int* __restrict__ outPointsIndexes,
                const float3* __restrict__ d_queries,
                const int     numQueries,
                const float3* __restrict__ d_nodes,
                const int     numNodes,
                const float   cutOffRadius)
{
    int tid = threadIdx.x+blockIdx.x*blockDim.x;
    if (tid >= numQueries) return;
    outPointsIndexes[tid*NUM_NEIGHBORS] = -1;

    CandidateList result(cutOffRadius);
    //traversal methods: stackBased, stackFree, cct
    float sqrDist = cukd::stackBased::knn<CandidateList, float3, cukd::default_data_traits<float3>>(result,
            d_queries[tid], d_nodes, numNodes);

    if (sqrDist < 1.0)
    { 
        for (auto i=0; i<NUM_NEIGHBORS; ++i)
            outPointsIndexes[tid*NUM_NEIGHBORS+i] = result.get_pointID(i);
    }

}

void run_kernel_corner(Eigen::Vector3d* outPoints,
                const float3 *d_queries,
                const int     numQueries,
                const float3* d_nodes,
                const int     numNodes,
                const float   cutOffRadius,
                const cudaStream_t stream)
{
    int bs = 128;
    int nb = static_cast<int>(std::ceil(static_cast<double>(numQueries) / bs));
//   CUKD_CUDA_SYNC_CHECK();

    d_knn_corner<cukd::FixedCandidateList<NUM_NEIGHBORS>><<<nb,bs,0,stream>>>
        (outPoints,
        d_queries,
        numQueries,
        d_nodes,
        numNodes,
        cutOffRadius);

  //CUKD_CUDA_SYNC_CHECK();
}

void run_kernel_surf(int* outPointsIndexes,
                const float3 *d_queries,
                const int     numQueries,
                const float3* d_nodes,
                const int     numNodes,
                const float   cutOffRadius,
                const cudaStream_t stream)
{
    int bs = 128;
    int nb = static_cast<int>(std::ceil(static_cast<double>(numQueries) / bs));
//   CUKD_CUDA_SYNC_CHECK();

    d_knn_surf<cukd::FixedCandidateList<NUM_NEIGHBORS>><<<nb,bs,0,stream>>>
        (outPointsIndexes,
        d_queries,
        numQueries,
        d_nodes,
        numNodes,
        cutOffRadius);

  //CUKD_CUDA_SYNC_CHECK();
}

