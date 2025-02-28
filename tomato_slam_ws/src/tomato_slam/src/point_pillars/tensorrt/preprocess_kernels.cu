// ~/tomato_slam_ws/src/tomato_slam/src/point_pillars/tensorrt/preprocess_kernels.cu
#include <cuda_runtime.h>

namespace tomato_slam {

// 声明核函数
__global__ void preprocessPointsKernel(const float* points, int num_points, 
                                      float* voxel_features, int* voxel_num_points,
                                      float min_x, float min_y, float min_z,
                                      float max_x, float max_y, float max_z,
                                      float voxel_x, float voxel_y, float voxel_z,
                                      int grid_x, int grid_y, int grid_z,
                                      int max_points_per_voxel, int feature_num) {
    // 实现点云预处理逻辑
    // ...
}

// 主机端函数包装
void launchPreprocessKernel(const float* points, int num_points,
                          float* voxel_features, int* voxel_num_points,
                          const float* voxel_params, const int* grid_params, 
                          int max_points_per_voxel, int feature_num,
                          cudaStream_t stream) {
    
    // 提取参数
    float min_x = voxel_params[0];
    float min_y = voxel_params[1];
    float min_z = voxel_params[2];
    float max_x = voxel_params[3];
    float max_y = voxel_params[4];
    float max_z = voxel_params[5];
    float voxel_x = voxel_params[6];
    float voxel_y = voxel_params[7];
    float voxel_z = voxel_params[8];
    
    int grid_x = grid_params[0];
    int grid_y = grid_params[1];
    int grid_z = grid_params[2];
    
    // 计算网格和块大小
    int block_size = 256;
    int grid_size = (num_points + block_size - 1) / block_size;
    
    // 启动内核
    preprocessPointsKernel<<<grid_size, block_size, 0, stream>>>(
        points, num_points, voxel_features, voxel_num_points,
        min_x, min_y, min_z, max_x, max_y, max_z,
        voxel_x, voxel_y, voxel_z, grid_x, grid_y, grid_z,
        max_points_per_voxel, feature_num);
}

} // namespace tomato_slam
