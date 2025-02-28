#pragma once

#include <memory>
#include <mutex>
#include <string>

// PCL包含
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Eigen包含
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV包含
#include <opencv2/core/core.hpp>

// ROS包含
#include <ros/ros.h>

// 直接包含ORB_SLAM3头文件，不要使用前向声明
#include "/home/yw/ORB_SLAM3/include/System.h"

namespace tomato_slam {

class OrbSlam3Interface {
public:
    // 修改构造函数
    OrbSlam3Interface(const std::string& vocFile, const std::string& settingsFile, 
                     ORB_SLAM3::System::eSensor sensorType = ORB_SLAM3::System::RGBD);
    
    ~OrbSlam3Interface();
    
    // 处理函数
    bool processRGBD(const cv::Mat& im, const cv::Mat& depthmap, double timestamp, Eigen::Matrix4f& pose);
    bool processMono(const cv::Mat& im, double timestamp, Eigen::Matrix4f& pose);
    bool processStereo(const cv::Mat& im_left, const cv::Mat& im_right, double timestamp, Eigen::Matrix4f& pose);
    
    void getCurrentMapPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& mapPointCloud);
    
    void shutdown();
    bool isTracking() const;
    Eigen::Matrix4f getCurrentPose() const;

private:
    // 使用原始指针而不是智能指针，与ORB_SLAM3接口一致
    ORB_SLAM3::System* mpSLAM;
    mutable std::mutex mMutex;
    bool mIsTracking;
    Eigen::Matrix4f mCurrentPose;
};

} // namespace tomato_slam
