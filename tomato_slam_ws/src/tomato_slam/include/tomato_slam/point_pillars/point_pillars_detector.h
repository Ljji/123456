// ~/tomato_slam_ws/src/tomato_slam/include/tomato_slam/point_pillars/point_pillars_detector.h
#pragma once

#include <string>
#include <vector>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

// 包含完整的TensorRTInference定义
#include <tomato_slam/point_pillars/tensorrt/tensorrt_inference.h>

namespace tomato_slam {

struct InternalTomatoDetection {
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
    Eigen::Vector3f dimensions;
    float confidence;
    float radius;
    int id;  // 添加ID字段
};

class PointPillarsDetector {
public:
    PointPillarsDetector(const std::string& enginePath,
                        float nmsThreshold = 0.3f,
                        float scoreThreshold = 0.5f);
    // 移除显式默认析构函数，避免与实现冲突
    ~PointPillarsDetector();
    
    bool detectTomatoes(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       std::vector<InternalTomatoDetection>& detections);
    
    bool detectTomatoes(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       const Eigen::Matrix4f& cameraPose,
                       std::vector<InternalTomatoDetection>& detections);

private:
    std::unique_ptr<TensorRTInference> mInference;
    
    // 确保变量名统一
    float mNmsThreshold;  // 使用小写的NMS
    float mScoreThreshold;
    
    // 修改函数原型以匹配实现
    void preprocessPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                           float* points_data, int& num_points);
    
    void parseDetections(const float* box_preds, const float* cls_preds,
                      const float* dir_preds, int num_detections,
                      std::vector<InternalTomatoDetection>& detections);
    
    void performNMS(std::vector<InternalTomatoDetection>& detections);
    
    void transformDetections(std::vector<InternalTomatoDetection>& detections,
                          const Eigen::Matrix4f& cameraPose);
};

} // namespace tomato_slam
