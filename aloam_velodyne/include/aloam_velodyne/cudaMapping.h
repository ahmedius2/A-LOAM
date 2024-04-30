#pragma once

#define NUM_NEIGHBORS 5

/**
 * Run this func once
 */
bool init_cuda();

void buildKDTree(float3 *refPoints, int numPoints);

void run_kernel_corner(Eigen::Vector3d* outPoints,
                const float3 *d_queries,
                const int     numQueries,
                const float3* d_nodes,
                const int     numNodes,
                const float   cutOffRadius = 1.0,
                const cudaStream_t stream = 0
                );

void run_kernel_surf(int* outPointsIndexes,
                const float3 *d_queries,
                const int     numQueries,
                const float3* d_nodes,
                const int     numNodes,
                const float   cutOffRadius = 1.0,
                const cudaStream_t stream = 0
                );
