#ifndef TOMATO_SLAM_DETECTION_VISUALIZER_H
#define TOMATO_SLAM_DETECTION_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include "tomato_slam/point_pillars/point_pillars_detector.h"

namespace tomato_slam {

class DetectionVisualizer {
public:
    DetectionVisualizer(ros::NodeHandle& nh, const std::string& topic = "tomato_markers");
    ~DetectionVisualizer();
    
    // 可视化检测结果
    void visualize(const std::vector<TomatoDetection>& detections, 
                  const std::string& frame_id, 
                  const ros::Time& timestamp = ros::Time::now());
    
    // 设置是否使用彩色编码置信度
    void setColorConfidence(bool enable) { mColorConfidence = enable; }
    
    // 设置标记的持久时间
    void setMarkerDuration(double duration) { mMarkerDuration = duration; }
    
    // 设置显示文本标签
    void setShowLabels(bool show) { mShowLabels = show; }
    
private:
    // ROS发布器
    ros::Publisher mMarkerPublisher;
    
    // 参数
    bool mColorConfidence;  // 是否根据置信度着色
    double mMarkerDuration; // 标记持久时间
    bool mShowLabels;       // 是否显示文本标签
    
    // 创建番茄标记
    visualization_msgs::Marker createTomatoMarker(
        const TomatoDetection& detection, 
        const std::string& frame_id,
        const ros::Time& timestamp,
        int id);
    
    // 创建文本标记
    visualization_msgs::Marker createTextMarker(
        const TomatoDetection& detection, 
        const std::string& frame_id,
        const ros::Time& timestamp,
        int id);
    
    // 获取基于置信度的颜色
    std_msgs::ColorRGBA getConfidenceColor(float confidence);
};

} // namespace tomato_slam

#endif // TOMATO_SLAM_DETECTION_VISUALIZER_H
