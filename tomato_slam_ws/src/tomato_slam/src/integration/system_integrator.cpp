#include "tomato_slam/integration/system_integrator.h"
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

namespace tomato_slam {

SystemIntegrator::SystemIntegrator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), initialized_(false)
{
    // 创建点云对象
    current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // 获取参数
    std::string orb_vocab_path, orb_settings_path;
    pnh_.getParam("orb_vocab_path", orb_vocab_path);
    pnh_.getParam("orb_settings_path", orb_settings_path);
    pnh_.getParam("point_pillars_engine_path", point_pillars_engine_path_);
    
    // 初始化ORB-SLAM3
    orb_slam_ = std::make_unique<OrbSlam3Interface>(orb_vocab_path, orb_settings_path);
    
    // 初始化PointPillars
    point_pillars_ = std::make_unique<PointPillarsDetector>(point_pillars_engine_path_);
    
    // 初始化可视化器
    visualizer_ = std::make_unique<DetectionVisualizer>(nh_);
    
    // 创建订阅
    rgb_sub_ = nh_.subscribe("camera/rgb/image_raw", 1,
                             &SystemIntegrator::rgbCallback, this);
    depth_sub_ = nh_.subscribe("camera/depth/image_raw", 1,
                              &SystemIntegrator::depthCallback, this);
    point_cloud_sub_ = nh_.subscribe("camera/depth/points", 1,
                                    &SystemIntegrator::pointCloudCallback, this);
    
    // 创建发布
    detection_pub_ = nh_.advertise<tomato_slam::TomatoDetections>("tomato_detections", 1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    
    ROS_INFO("System integrator initialized");
    initialized_ = true;
}

SystemIntegrator::~SystemIntegrator()
{
    if (orb_slam_) {
        orb_slam_->shutdown();
    }
}

void SystemIntegrator::rgbCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        current_rgb_ = cv_ptr->image;
        rgb_timestamp_ = msg->header.stamp;
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("CV bridge exception: %s", e.what());
    }
}

void SystemIntegrator::depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
        current_depth_ = cv_ptr->image;
        depth_timestamp_ = msg->header.stamp;
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("CV bridge exception: %s", e.what());
    }
}

void SystemIntegrator::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!initialized_) return;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);
    current_cloud_ = cloud;
    cloud_timestamp_ = msg->header.stamp;
    
    // 只在有跟踪时检测番茄
    if (orb_slam_->isTracking()) {
        Eigen::Matrix4f pose = orb_slam_->getCurrentPose();
        
        // 转换XYZRGB点云到XYZ点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        xyz_cloud->points.reserve(cloud->points.size());
        
        for (const auto& pt : cloud->points) {
            pcl::PointXYZ xyz_pt;
            xyz_pt.x = pt.x;
            xyz_pt.y = pt.y;
            xyz_pt.z = pt.z;
            xyz_cloud->push_back(xyz_pt);
        }
        
        // 检测番茄
        std::vector<InternalTomatoDetection> detections;
        point_pillars_->detectTomatoes(xyz_cloud, pose, detections);
        
        // 发布检测结果
        tomato_slam::TomatoDetections detection_msg;
        detection_msg.header = msg->header;
        
        for (const auto& det : detections) {
            tomato_slam::TomatoDetection single_det;
            single_det.position.x = det.position.x();
            single_det.position.y = det.position.y();
            single_det.position.z = det.position.z();
            single_det.radius = det.radius;
            single_det.confidence = det.confidence;
            
            detection_msg.detections.push_back(single_det);
        }
        
        detection_pub_.publish(detection_msg);
        
        // 可视化检测结果
        visualizer_->visualizeDetections(detections, pose, msg->header.frame_id);
    }
}

void SystemIntegrator::processFrames()
{
    if (current_rgb_.empty() || current_depth_.empty()) {
        return;
    }
    
    // 检查RGB和深度图像的时间戳
    double time_diff = std::abs((rgb_timestamp_ - depth_timestamp_).toSec());
    if (time_diff > 0.1) { // 100ms阈值
        ROS_WARN("RGB and depth timestamps differ by %.3f seconds", time_diff);
        return;
    }
    
    // 处理RGB-D帧
    Eigen::Matrix4f pose;
    orb_slam_->processRGBD(current_rgb_, current_depth_, rgb_timestamp_.toSec(), pose);
    
    // 如果跟踪成功，发布位姿转换
    if (orb_slam_->isTracking()) {
        Eigen::Matrix4f pose = orb_slam_->getCurrentPose();
        
        // 提取旋转矩阵和平移向量
        Eigen::Matrix3f rot = pose.block<3,3>(0,0);
        Eigen::Vector3f trans = pose.block<3,1>(0,3);
        
        // 构造转换消息
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "camera_link";
        
        // 设置平移
        transform_stamped.transform.translation.x = trans.x();
        transform_stamped.transform.translation.y = trans.y();
        transform_stamped.transform.translation.z = trans.z();
        
        // 从旋转矩阵创建四元数
        Eigen::Quaternionf q(rot);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        
        // 发布转换
        tf_broadcaster_->sendTransform(transform_stamped);
    }
}

} // namespace tomato_slam
