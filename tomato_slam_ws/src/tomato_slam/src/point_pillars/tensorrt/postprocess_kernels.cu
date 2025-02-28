#include <cuda_runtime_api.h>
#include <iostream>

namespace tomato_slam {

constexpr int THREADS_PER_BLOCK = 256;

// 设备常量
__constant__ float score_threshold;
__constant__ int num_classes;
__constant__ int num_dir_bins;
__constant__ float dir_offset;
__constant__ float dir_limit_offset;
__constant__ float nms_threshold;
__constant__ int max_detections;

// 筛选检测结果
__global__ void filter_detections_kernel(const float* box_preds, 
                                       const float* cls_preds, 
                                       const float* dir_preds,
                                       int num_boxes,
                                       float* filtered_boxes,
                                       float* filtered_scores,
                                       float* filtered_dirs,
                                       int* num_filtered) {
    int box_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (box_idx >= num_boxes) return;

    // 获取当前框的类别得分
    float score = 0.0f;
    int class_idx = 0;
    for (int c = 0; c < num_classes; ++c) {
        float cls_score = cls_preds[box_idx * num_classes + c];
        if (cls_score > score) {
            score = cls_score;
            class_idx = c;
        }
    }
    
    // 应用置信度阈值
    if (score < score_threshold) return;
    
    // 获取方向预测
    float dir_score_0 = dir_preds[box_idx * num_dir_bins + 0];
    float dir_score_1 = dir_preds[box_idx * num_dir_bins + 1];
    int dir_label = dir_score_0 > dir_score_1 ? 0 : 1;
    
    // 添加到过滤后的结果
    int idx = atomicAdd(num_filtered, 1);
    if (idx < max_detections) {
        // 复制框预测 (7个值: x, y, z, w, l, h, yaw)
        for (int i = 0; i < 7; ++i) {
            filtered_boxes[idx * 7 + i] = box_preds[box_idx * 7 + i];
        }
        
        // 如果方向标签为1，则旋转180度
        if (dir_label == 1) {
            float yaw = filtered_boxes[idx * 7 + 6];
            filtered_boxes[idx * 7 + 6] = yaw + 3.14159265f;
        }
        
        // 存储得分和类别
        filtered_scores[idx] = score;
        filtered_dirs[idx] = dir_label;
    }
}

// 执行后处理（筛选和方向修正）
int filterDetections(const float* box_preds, 
                    const float* cls_preds, 
                    const float* dir_preds,
                    int num_boxes,
                    float* filtered_boxes,
                    float* filtered_scores,
                    float* filtered_dirs,
                    int* dev_num_filtered) {
    // 初始化检测数量为0
    cudaMemset(dev_num_filtered, 0, sizeof(int));
    
    // 计算并设置启动配置
    int num_blocks = (num_boxes + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    
    // 调用筛选核函数
    filter_detections_kernel<<<num_blocks, THREADS_PER_BLOCK>>>(
        box_preds, cls_preds, dir_preds, num_boxes,
        filtered_boxes, filtered_scores, filtered_dirs, dev_num_filtered);
    
    // 同步
    cudaDeviceSynchronize();
    
    // 获取过滤后的检测数量
    int num_filtered = 0;
    cudaMemcpy(&num_filtered, dev_num_filtered, sizeof(int), cudaMemcpyDeviceToHost);
    
    return num_filtered;
}

// 设置后处理参数
void setPostprocessParams(float score_thresh, int num_cls, int num_dir_bin,
                         float dir_off, float dir_limit_off, float nms_thresh, int max_det) {
    cudaMemcpyToSymbol(score_threshold, &score_thresh, sizeof(float));
    cudaMemcpyToSymbol(num_classes, &num_cls, sizeof(int));
    cudaMemcpyToSymbol(num_dir_bins, &num_dir_bin, sizeof(int));
    cudaMemcpyToSymbol(dir_offset, &dir_off, sizeof(float));
    cudaMemcpyToSymbol(dir_limit_offset, &dir_limit_off, sizeof(float));
    cudaMemcpyToSymbol(nms_threshold, &nms_thresh, sizeof(float));
    cudaMemcpyToSymbol(max_detections, &max_det, sizeof(int));
}

} // namespace tomato_slam
