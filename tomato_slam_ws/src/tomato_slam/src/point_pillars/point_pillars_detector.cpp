// ~/tomato_slam_ws/src/tomato_slam/src/point_pillars/point_pillars_detector.cpp
#include <tomato_slam/point_pillars/point_pillars_detector.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>
#include <ros/ros.h>

namespace tomato_slam {

PointPillarsDetector::PointPillarsDetector(const std::string& enginePath,
                                        float nmsThreshold,
                                        float scoreThreshold)
    : mNmsThreshold(nmsThreshold),  // 确保使用正确的成员变量名
      mScoreThreshold(scoreThreshold)
{
    mInference = std::make_unique<TensorRTInference>(enginePath);
    
    ROS_INFO("PointPillars detector initialized with NMS threshold: %.2f, score threshold: %.2f",
              mNmsThreshold, mScoreThreshold);  // 使用正确的成员变量名
}

// 定义析构函数，不再使用=default
PointPillarsDetector::~PointPillarsDetector() {
    // 空实现
}

bool PointPillarsDetector::detectTomatoes(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                        std::vector<InternalTomatoDetection>& detections) {  // 修正类型
    if (cloud->empty()) {
        return false;
    }
    
    // 预处理点云
    const int maxPoints = mInference->getMaxInputPoints();
    std::vector<float> points_data(maxPoints * 4);  // x, y, z, intensity
    int num_points = 0;
    
    preprocessPointCloud(cloud, points_data.data(), num_points);
    
    if (num_points == 0) {
        return false;
    }
    
    // 运行推理
    const int maxDetections = mInference->getMaxDetections();
    const int numClasses = mInference->getNumClasses();
    
    std::vector<float> box_preds(maxDetections * 7);  // x, y, z, w, l, h, theta
    std::vector<float> cls_preds(maxDetections * numClasses);
    std::vector<float> dir_preds(maxDetections * 2);  // sin, cos
    
    bool success = mInference->infer(points_data.data(), num_points, box_preds.data(), cls_preds.data(), dir_preds.data());
    
    if (!success) {
        ROS_ERROR("TensorRT inference failed");
        return false;
    }
    
    // 解析检测结果
    parseDetections(box_preds.data(), cls_preds.data(), dir_preds.data(), maxDetections, detections);
    
    // 应用NMS
    performNMS(detections);
    
    return !detections.empty();
}

// 使用正确的参数类型
void PointPillarsDetector::preprocessPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                             float* points_data, int& num_points) {
    // 转换点云到PointPillars输入格式
    num_points = 0;
    for (const auto& point : cloud->points) {
        if (num_points >= mInference->getMaxInputPoints()) {
            break;
        }
        
        // 过滤无效点
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        
        // 将点转换为PointPillars输入格式: [x, y, z, intensity]
        points_data[num_points * 4 + 0] = point.x;
        points_data[num_points * 4 + 1] = point.y;
        points_data[num_points * 4 + 2] = point.z;
        points_data[num_points * 4 + 3] = 1.0f;  // 使用1.0作为强度值
        
        num_points++;
    }
}

// 使用正确的类型
void PointPillarsDetector::parseDetections(const float* box_preds,
                                        const float* cls_preds,
                                        const float* dir_preds,
                                        int num_detections,
                                        std::vector<InternalTomatoDetection>& detections) {  // 正确的类型
    detections.clear();
    
    for (int i = 0; i < num_detections; i++) {
        // 获取分类得分
        float score = cls_preds[i];
        
        // 应用分数阈值
        if (score < mScoreThreshold) {
            continue;
        }
        
        // 提取检测框参数: [x, y, z, w, l, h, theta]
        float x = box_preds[i * 7 + 0];
        float y = box_preds[i * 7 + 1];
        float z = box_preds[i * 7 + 2];
        float width = box_preds[i * 7 + 3];
        float length = box_preds[i * 7 + 4];
        float height = box_preds[i * 7 + 5];
        float theta = box_preds[i * 7 + 6];
        
        // 应用方向预测
        float sin_t = dir_preds[i * 2 + 0];
        float cos_t = dir_preds[i * 2 + 1];
        float dir_score = std::abs(cos_t) > std::abs(sin_t) ? cos_t : sin_t;
        theta += (dir_score > 0 ? 0 : M_PI);
        
        // 创建检测结果对象
        InternalTomatoDetection detection;  // 正确的类型
        detection.id = i;
        detection.position = Eigen::Vector3f(x, y, z);
        
        // 从欧拉角创建四元数
        Eigen::AngleAxisf rotation(theta, Eigen::Vector3f::UnitZ());
        detection.orientation = Eigen::Quaternionf(rotation);
        
        detection.dimensions = Eigen::Vector3f(width, length, height);
        detection.confidence = score;
        
        // 计算番茄的半径 (简化为dimensions的平均值除以2)
        detection.radius = (width + length + height) / 6.0f;
        
        detections.push_back(detection);
    }
}

// 修正类型
void PointPillarsDetector::performNMS(std::vector<InternalTomatoDetection>& detections) {  // 使用InternalTomatoDetection
    if (detections.size() <= 1) {
        return;
    }
    
    // 按置信度排序
    std::sort(detections.begin(), detections.end(),
              [](const InternalTomatoDetection& a, const InternalTomatoDetection& b) {
                  return a.confidence > b.confidence;
              });
    
    std::vector<InternalTomatoDetection> nms_result;
    std::vector<bool> is_suppressed(detections.size(), false);
    
    for (size_t i = 0; i < detections.size(); i++) {
        if (is_suppressed[i]) {
            continue;
        }
        
        nms_result.push_back(detections[i]);
        
        for (size_t j = i + 1; j < detections.size(); j++) {
            if (is_suppressed[j]) {
                continue;
            }
            
            // 计算两个检测框中心点的距离
            Eigen::Vector3f dist_vec = detections[i].position - detections[j].position;
            float dist = dist_vec.norm();
            
            // 计算两个检测框的平均半径
            float avg_radius = (detections[i].radius + detections[j].radius) / 2.0f;
            
            // 如果距离小于NMS阈值，则抑制检测框
            if (dist < avg_radius * mNmsThreshold) {  // 使用正确的成员变量名
                is_suppressed[j] = true;
            }
        }
    }
    
    detections = std::move(nms_result);
}

// 修正类型
void PointPillarsDetector::transformDetections(std::vector<InternalTomatoDetection>& detections,  // 使用InternalTomatoDetection
                                            const Eigen::Matrix4f& cameraPose) {
    for (auto& detection : detections) {
        // 获取检测位置
        Eigen::Vector4f pos_local(detection.position.x(), detection.position.y(), detection.position.z(), 1.0f);
        
        // 转换到世界坐标系
        Eigen::Vector4f pos_world = cameraPose * pos_local;
        detection.position = pos_world.head<3>();
        
        // 转换方向四元数
        Eigen::Matrix3f rot_local = detection.orientation.toRotationMatrix();
        Eigen::Matrix3f rot_camera = cameraPose.block<3,3>(0,0);
        Eigen::Matrix3f rot_world = rot_camera * rot_local;
        detection.orientation = Eigen::Quaternionf(rot_world);
    }
}

// 实现第二个重载版本的detectTomatoes
bool PointPillarsDetector::detectTomatoes(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                       const Eigen::Matrix4f& cameraPose,
                                       std::vector<InternalTomatoDetection>& detections) {
    if (!detectTomatoes(cloud, detections)) {
        return false;
    }
    
    // 将检测结果转换到世界坐标系
    transformDetections(detections, cameraPose);
    
    return true;
}

} // namespace tomato_slam
