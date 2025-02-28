
#ifndef CONVERSION_UTILS_H
#define CONVERSION_UTILS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include "tomato_slam/TomatoDetection.h"
#include "tomato_slam/point_pillars/point_pillars_detector.h"

namespace tomato_slam {

// Eigen转ROS
inline geometry_msgs::Point eigenToROSPoint(const Eigen::Vector3f& v) {
    geometry_msgs::Point p;
    p.x = v(0);
    p.y = v(1);
    p.z = v(2);
    return p;
}

inline geometry_msgs::Vector3 eigenToROSVector3(const Eigen::Vector3f& v) {
    geometry_msgs::Vector3 vec;
    vec.x = v(0);
    vec.y = v(1);
    vec.z = v(2);
    return vec;
}

inline geometry_msgs::Quaternion eigenToROSQuaternion(const Eigen::Quaternionf& q) {
    geometry_msgs::Quaternion quat;
    quat.w = q.w();
    quat.x = q.x();
    quat.y = q.y();
    quat.z = q.z();
    return quat;
}

// ROS转Eigen
inline Eigen::Vector3f ROSToEigenVector3(const geometry_msgs::Point& p) {
    return Eigen::Vector3f(p.x, p.y, p.z);
}

inline Eigen::Vector3f ROSToEigenVector3(const geometry_msgs::Vector3& v) {
    return Eigen::Vector3f(v.x, v.y, v.z);
}

inline Eigen::Quaternionf ROSToEigenQuaternion(const geometry_msgs::Quaternion& q) {
    return Eigen::Quaternionf(q.w, q.x, q.y, q.z);
}

// 转换内部检测结构体到ROS消息
inline TomatoDetection toRosMsg(const InternalTomatoDetection& detection) {
    TomatoDetection msg;
    
    msg.position = eigenToROSPoint(detection.position);
    msg.dimensions = eigenToROSVector3(detection.dimensions);
    msg.orientation = eigenToROSQuaternion(detection.orientation);
    msg.confidence = detection.confidence;
    msg.class_name = detection.class_name;
    
    return msg;
}

// 从ROS消息转换到内部检测结构体
inline InternalTomatoDetection fromRosMsg(const TomatoDetection& msg) {
    InternalTomatoDetection detection;
    
    detection.position = ROSToEigenVector3(msg.position);
    detection.dimensions = ROSToEigenVector3(msg.dimensions);
    detection.orientation = ROSToEigenQuaternion(msg.orientation);
    detection.confidence = msg.confidence;
    detection.class_name = msg.class_name;
    
    return detection;
}

// 变换检测结果
inline void transformDetection(TomatoDetection& detection, const Eigen::Matrix4f& transform) {
    // 转换为Eigen类型
    Eigen::Vector3f position = ROSToEigenVector3(detection.position);
    Eigen::Quaternionf orientation = ROSToEigenQuaternion(detection.orientation);
    
    // 创建齐次坐标
    Eigen::Vector4f pos(position.x(), position.y(), position.z(), 1.0f);
    
    // 应用变换
    pos = transform * pos;
    
    // 提取旋转部分
    Eigen::Matrix3f rot = transform.block<3,3>(0,0);
    Eigen::Quaternionf new_orientation(rot * orientation.toRotationMatrix());
    
    // 转回ROS类型
    detection.position = eigenToROSPoint(pos.head<3>());
    detection.orientation = eigenToROSQuaternion(new_orientation);
}

} // namespace tomato_slam

#endif // CONVERSION_UTILS_H
