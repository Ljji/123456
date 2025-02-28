// ~/tomato_slam_ws/src/tomato_slam/src/visualization/detection_visualizer.cpp
#include <tomato_slam/visualization/detection_visualizer.h>

namespace tomato_slam {

// 确保在初始化列表中初始化引用成员
DetectionVisualizer::DetectionVisualizer(ros::NodeHandle& nh, const std::string& topic)
    : mNodeHandle(nh), mTopic(topic)
{
    mMarkerPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(mTopic, 1);
}

void DetectionVisualizer::visualizeDetections(const std::vector<InternalTomatoDetection>& detections,
                                          const Eigen::Matrix4f& cameraPose,
                                          const std::string& frame_id)
{
    visualization_msgs::MarkerArray marker_array;
    ros::Time now = ros::Time::now();
    
    for (size_t i = 0; i < detections.size(); i++) {
        marker_array.markers.push_back(createTomatoMarker(detections[i], frame_id, now, i * 2));
        marker_array.markers.push_back(createTextMarker(detections[i], frame_id, now, i * 2 + 1));
    }
    
    mMarkerPublisher.publish(marker_array);
}

visualization_msgs::Marker DetectionVisualizer::createTomatoMarker(const InternalTomatoDetection& detection,
                                                               const std::string& frame_id,
                                                               const ros::Time& timestamp,
                                                               int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = timestamp;
    marker.ns = "tomatoes";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 修复点调用方式
    marker.pose.position.x = detection.position.x();
    marker.pose.position.y = detection.position.y();
    marker.pose.position.z = detection.position.z();
    
    // 修复四元数调用方式
    marker.pose.orientation.x = detection.orientation.x();
    marker.pose.orientation.y = detection.orientation.y();
    marker.pose.orientation.z = detection.orientation.z();
    marker.pose.orientation.w = detection.orientation.w();
    
    // 使用直接访问而不是调用方法
    float radius = detection.radius;
    
    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = radius * 2;
    
    // 根据置信度设置颜色
    marker.color.r = 1.0f - detection.confidence;
    marker.color.g = detection.confidence;
    marker.color.b = 0.0f;
    marker.color.a = 0.7f;
    
    marker.lifetime = ros::Duration(0.5);
    
    return marker;
}

visualization_msgs::Marker DetectionVisualizer::createTextMarker(const InternalTomatoDetection& detection,
                                                             const std::string& frame_id,
                                                             const ros::Time& timestamp,
                                                             int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = timestamp;
    marker.ns = "tomato_labels";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 使用直接访问而不是调用方法
    float radius = detection.radius;
    
    // 修复点调用方式
    marker.pose.position.x = detection.position.x();
    marker.pose.position.y = detection.position.y();
    marker.pose.position.z = detection.position.z() + radius * 2;
    
    marker.pose.orientation.w = 1.0;
    
    marker.scale.z = 0.1;  // 文本大小
    
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.8f;
    
    // 格式化置信度
    char confidence_str[32];
    sprintf(confidence_str, "%.2f", detection.confidence);
    marker.text = confidence_str;
    
    marker.lifetime = ros::Duration(0.5);
    
    return marker;
}

} // namespace tomato_slam
