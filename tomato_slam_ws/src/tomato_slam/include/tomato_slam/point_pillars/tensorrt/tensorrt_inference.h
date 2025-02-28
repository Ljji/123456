// ~/tomato_slam_ws/src/tomato_slam/include/tomato_slam/point_pillars/tensorrt/tensorrt_inference.h
#pragma once

#include <string>
#include <vector>
#include <memory>
#include <iostream>  // 添加
#include <unordered_map>  // 添加
#include <fstream>  // 添加
#include <NvInfer.h>

namespace tomato_slam {

// 添加日志记录器类，修复noexcept
class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity != Severity::kINFO) {
            std::cout << msg << std::endl;
        }
    }
};

class TensorRTInference {
public:
    TensorRTInference(const std::string& engine_path);
    ~TensorRTInference();
    
    // 现有函数
    bool infer(float* points_data, int num_points,
               float* box_preds, float* cls_preds, float* dir_preds);
    
    // 添加缺失的函数声明
    void setupBuffers();
    void infer(); // 重载版本
    void* getBuffer(const std::string& name);
    size_t getBufferSize(const std::string& name);
    
    // 添加辅助函数
    int getMaxInputPoints() const { return max_input_points_; }
    int getMaxDetections() const { return max_detections_; }
    int getNumClasses() const { return num_classes_; }

private:
    std::unique_ptr<Logger> logger_;
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    
    std::vector<void*> device_buffers_;
    std::vector<void*> host_buffers_;
    std::unordered_map<std::string, int> binding_map_;
    std::unordered_map<std::string, size_t> binding_sizes_;
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    
    // 配置参数
    int max_input_points_ = 20000;
    int max_detections_ = 100;
    int num_classes_ = 1;  // 只检测番茄
};

} // namespace tomato_slam
