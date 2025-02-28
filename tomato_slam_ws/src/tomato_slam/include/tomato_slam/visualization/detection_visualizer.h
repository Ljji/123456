// ~/tomato_slam_ws/src/tomato_slam/include/tomato_slam/visualization/detection_visualizer.h
#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <Eigen/Core>
#include <tomato_slam/point_pillars/point_pillars_detector.h>

namespace tomato_slam {

class DetectionVisualizer {
public:
    // 修改构造函数，使用指针而不是引用，或在初始化列表中初始化引用
    DetectionVisualizer(ros::NodeHandle& nh, const std::string& topic = "tomato_markers");
    
    void visualizeDetections(const std::vector<InternalTomatoDetection>& detections,
                           const Eigen::Matrix4f& cameraPose,
                           const std::string& frame_id);

private:
    visualization_msgs::Marker createTomatoMarker(const InternalTomatoDetection& detection,
                                              const std::string& frame_id,
                                              const ros::Time& timestamp,
                                              int id);
    
    visualization_msgs::Marker createTextMarker(const InternalTomatoDetection& detection,
                                            const std::string& frame_id,
                                            const ros::Time& timestamp,
                                            int id);
    
    // 修改为引用的正确初始化
    ros::NodeHandle& mNodeHandle;
    ros::Publisher mMarkerPublisher;
    std::string mTopic;
};

} // namespace tomato_slam
