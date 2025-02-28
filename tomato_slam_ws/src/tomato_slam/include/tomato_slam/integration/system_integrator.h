#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <mutex>
#include <memory>
#include <opencv2/core/core.hpp>

// PCL包含
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// 添加cv_bridge包含
#include <cv_bridge/cv_bridge.h>

// 添加消息类型
#include <tomato_slam/TomatoDetection.h>
#include <tomato_slam/TomatoDetections.h>

#include "tomato_slam/orb_slam3/orb_slam3_interface.h"
#include "tomato_slam/point_pillars/point_pillars_detector.h"
#include "tomato_slam/visualization/detection_visualizer.h"
#include "tomato_slam/common/bounding_box.h"

namespace tomato_slam {

class SystemIntegrator {
public:
    SystemIntegrator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~SystemIntegrator();

    void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void processFrames();

private:
    // 添加ROS节点句柄
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // 添加ROS订阅者
    ros::Subscriber rgb_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber point_cloud_sub_;
    
    std::unique_ptr<OrbSlam3Interface> orb_slam_;
    std::unique_ptr<PointPillarsDetector> point_pillars_;
    std::unique_ptr<DetectionVisualizer> visualizer_;
    
    // 添加其他必要变量
    bool initialized_;
    std::mutex mutex_;
    
    // 图像数据
    cv::Mat current_rgb_;
    cv::Mat current_depth_;
    ros::Time rgb_timestamp_;
    ros::Time depth_timestamp_;
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud_;
    ros::Time cloud_timestamp_;
    
    // 点云引擎路径
    std::string point_pillars_engine_path_;
    
    // 转换广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 发布器
    ros::Publisher detection_pub_;
};

} // namespace tomato_slam
