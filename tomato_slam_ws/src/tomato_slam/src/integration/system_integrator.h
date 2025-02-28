
#define BOOST_THREAD_PROVIDES_THREAD_COMPATIBILITY_1_0
#define BOOST_THREAD_VERSION 4
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#define BOOST_NO_CXX11_SCOPED_ENUMS
#define BOOST_FILESYSTEM_NO_DEPRECATED
#define BOOST_ALLOW_DEPRECATED_HEADERS

#include <boost/thread.hpp>
#ifndef TOMATO_SLAM_SYSTEM_INTEGRATOR_H
#define TOMATO_SLAM_SYSTEM_INTEGRATOR_H

// 在最顶部添加这些定义
#define BOOST_THREAD_PROVIDES_THREAD_COMPATIBILITY_1_0
#define BOOST_THREAD_VERSION 4
#include <boost/thread.hpp>

// 其他包含
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <mutex>
// 删除 #include <thread> 行，因为我们现在使用 boost::thread
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "tomato_slam/TomatoDetections.h"
#include "tomato_slam/orb_slam3/orb_slam3_interface.h"
#include "tomato_slam/point_pillars/point_pillars_detector.h"

namespace tomato_slam {

SystemIntegrator::SystemIntegrator(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : mNH(nh),
      mPNH(pnh),
      mLatestPointCloud(new pcl::PointCloud<pcl::PointXYZ>()),
      mRunning(false),
      mImageSyncSubscriber(nullptr),
      mDepthSyncSubscriber(nullptr),
      mRGBDSynchronizer(nullptr) {
    
    // 初始化为单位矩阵
    mCurrentPose = Eigen::Matrix4f::Identity();
    
    // 从参数服务器加载参数
    mPNH.param<std::string>("camera_frame_id", mCameraFrameId, "camera_link");
    mPNH.param<std::string>("map_frame_id", mMapFrameId, "map");
    mPNH.param<double>("detection_rate", mDetectionRate, 5.0);  // Hz
    mPNH.param<bool>("visualize_detections", mVisualizeDetections, true);
    
    ROS_INFO("System integrator initialized with detection rate: %.2f Hz", mDetectionRate);
}

SystemIntegrator::~SystemIntegrator() {
    mRunning = false;
    
    if (mSLAMThread.joinable()) {
        mSLAMThread.join();
    }
    
    if (mDetectionThread.joinable()) {
        mDetectionThread.join();
    }
    
    if (mImageSyncSubscriber) delete mImageSyncSubscriber;
    if (mDepthSyncSubscriber) delete mDepthSyncSubscriber;
    if (mRGBDSynchronizer) delete mRGBDSynchronizer;
}

bool SystemIntegrator::initialize() {
    ROS_INFO("Initializing system integrator...");
    
    // 创建发布者
    mPosePublisher = mNH.advertise<geometry_msgs::PoseStamped>("camera_pose", 1);
    mMapPointsPublisher = mNH.advertise<sensor_msgs::PointCloud2>("map_points", 1);
    mTomatoDetectionsPublisher = mNH.advertise<tomato_slam::TomatoDetections>("tomato_detections", 1);
    
    // 获取ORB-SLAM3配置参数
    std::string vocFile, settingsFile;
    mPNH.param<std::string>("orb_slam3/voc_file", vocFile, "");
    mPNH.param<std::string>("orb_slam3/settings_file", settingsFile, "");
    
    if (vocFile.empty() || settingsFile.empty()) {
        ROS_ERROR("Missing ORB-SLAM3 configuration files");
        return false;
    }
    
    // 初始化ORB-SLAM3
    try {
        mSLAM = std::make_unique<OrbSlam3Interface>(
            vocFile, settingsFile, ORB_SLAM3::System::RGBD);
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize ORB-SLAM3: %s", e.what());
        return false;
    }
    
    // 获取PointPillars配置参数
    std::string engineFile;
    float nmsThreshold, scoreThreshold;
    mPNH.param<std::string>("point_pillars/engine_file", engineFile, "");
    mPNH.param<float>("point_pillars/nms_threshold", nmsThreshold, 0.5f);
    mPNH.param<float>("point_pillars/score_threshold", scoreThreshold, 0.3f);
    
    if (engineFile.empty()) {
        ROS_ERROR("Missing PointPillars engine file");
        return false;
    }
    
    // 初始化PointPillars检测器
    try {
        mDetector = std::make_unique<PointPillarsDetector>(
            engineFile, nmsThreshold, scoreThreshold);
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize PointPillars detector: %s", e.what());
        return false;
    }
    
    // 设置同步的RGB-D订阅者
    mImageSyncSubscriber = new message_filters::Subscriber<sensor_msgs::Image>(
        mNH, "camera/rgb/image_raw", 1);
    mDepthSyncSubscriber = new message_filters::Subscriber<sensor_msgs::Image>(
        mNH, "camera/depth/image_raw", 1);
    
    mRGBDSynchronizer = new message_filters::Synchronizer<RGBDSyncPolicy>(
        RGBDSyncPolicy(10), *mImageSyncSubscriber, *mDepthSyncSubscriber);
    mRGBDSynchronizer->registerCallback(
        boost::bind(&SystemIntegrator::rgbdCallback, this, _1, _2));
    
    // 订阅点云话题
    mPointCloudSubscriber = mNH.subscribe(
        "camera/depth/points", 1, &SystemIntegrator::pointCloudCallback, this);
    
    ROS_INFO("System integrator initialized successfully");
    return true;
}

void SystemIntegrator::run() {
    mRunning = true;
    
    // 修改: 使用boost::thread代替std::thread
    mSLAMThread = boost::thread(&SystemIntegrator::slamThreadFunc, this);
    
    // 修改: 使用boost::thread代替std::thread
    mDetectionThread = boost::thread(&SystemIntegrator::detectionThreadFunc, this);
    
    ROS_INFO("System integrator running");
}

const std::vector<TomatoDetection>& SystemIntegrator::getTomatoDetections() const {
    std::lock_guard<std::mutex> lock(mDetectionMutex);
    return mDetections;
}

Eigen::Matrix4f SystemIntegrator::getCurrentPose() const {
    std::lock_guard<std::mutex> lock(mPoseMutex);
    return mCurrentPose;
}

void SystemIntegrator::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    
    std::lock_guard<std::mutex> lock(mPointCloudMutex);
    mLatestPointCloud = cloud;
}

void SystemIntegrator::rgbdCallback(const sensor_msgs::Image::ConstPtr& rgb_msg,
                                   const sensor_msgs::Image::ConstPtr& depth_msg) {
    try {
        // 转换RGB图像
        cv_bridge::CvImageConstPtr cv_rgb = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
        
        // 转换深度图像
        cv_bridge::CvImageConstPtr cv_depth = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        
        // 将16位深度图转换为32位浮点数
        cv::Mat depth_float;
        cv_depth->image.convertTo(depth_float, CV_32F, 1.0 / 1000.0);  // 假设单位是毫米，转换为米
        
        // 获取时间戳
        double timestamp = rgb_msg->header.stamp.toSec();
        
        // 处理RGBD图像
        Eigen::Matrix4f pose;
        bool success = mSLAM->processRGBD(cv_rgb->image, depth_float, timestamp, pose);
        
        if (success) {
            std::lock_guard<std::mutex> lock(mPoseMutex);
            mCurrentPose = pose;
            
            // 发布位姿
            publishPose(pose, rgb_msg->header.stamp);
            
            // 获取并发布地图点云
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
            mSLAM->getCurrentMapPoints(mapPoints);
            publishMapPoints(mapPoints, rgb_msg->header.stamp);
        }
        
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("CV Bridge error: %s", e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("Error processing RGB-D data: %s", e.what());
    }
}

void SystemIntegrator::slamThreadFunc() {
    ROS_INFO("SLAM thread started");
    
    ros::Rate rate(30);  // 30 Hz
    
    while (mRunning && ros::ok()) {
        // SLAM处理已经在回调函数中完成
        rate.sleep();
    }
    
    ROS_INFO("SLAM thread stopped");
}

void SystemIntegrator::detectionThreadFunc() {
    ROS_INFO("Detection thread started");
    
    ros::Rate rate(mDetectionRate);
    
    while (mRunning && ros::ok()) {
        try {
            // 获取最新的点云和相机位姿
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
            Eigen::Matrix4f pose;
            
            {
                std::lock_guard<std::mutex> lock1(mPointCloudMutex);
                std::lock_guard<std::mutex> lock2(mPoseMutex);
                
                if (mLatestPointCloud->empty() || !mSLAM->isTracking()) {
                    rate.sleep();
                    continue;
                }
                
                cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(*mLatestPointCloud));
                pose = mCurrentPose;
            }
            
            // 执行番茄检测
            std::vector<TomatoDetection> detections;
            bool success = mDetector->detectTomatoes(cloud, detections);
            
            if (success) {
                // 将检测结果从相机坐标系转换到世界坐标系
                mDetector->transformDetections(detections, pose);
                
                {
                    std::lock_guard<std::mutex> lock(mDetectionMutex);
                    mDetections = detections;
                }
                
                // 发布检测结果
                publishTomatoDetections(detections, ros::Time::now());
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR("Error in detection thread: %s", e.what());
        }
        
        rate.sleep();
    }
    
    ROS_INFO("Detection thread stopped");
}

void SystemIntegrator::publishPose(const Eigen::Matrix4f& pose, const ros::Time& timestamp) {
    // 创建位姿消息
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = timestamp;
    pose_msg.header.frame_id = mMapFrameId;
    
    // 将Eigen位姿转换为ROS位姿
    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose.cast<double>();
    tf::poseEigenToMsg(pose_affine, pose_msg.pose);
    
    // 发布位姿
    mPosePublisher.publish(pose_msg);
    
    // 发布TF变换
    tf::Transform transform;
    tf::poseMsgToTF(pose_msg.pose, transform);
    mTfBroadcaster.sendTransform(tf::StampedTransform(
        transform, timestamp, mMapFrameId, mCameraFrameId));
}

void SystemIntegrator::publishMapPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& mapPoints,
                                       const ros::Time& timestamp) {
    if (mapPoints->empty()) {
        return;
    }
    
    // 创建点云消息
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*mapPoints, cloud_msg);
    
    cloud_msg.header.stamp = timestamp;
    cloud_msg.header.frame_id = mMapFrameId;
    
    // 发布点云
    mMapPointsPublisher.publish(cloud_msg);
}

void SystemIntegrator::publishTomatoDetections(const std::vector<TomatoDetection>& detections,
                                             const ros::Time& timestamp) {
    if (detections.empty()) {
        return;
    }
    
    // 创建检测消息
    tomato_slam::TomatoDetections msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = mMapFrameId;
    
    // 添加所有检测结果
    for (const auto& det : detections) {
        tomato_slam::TomatoDetection det_msg;
        det_msg.id = det.id;
        det_msg.class_name = det.class_name;
        det_msg.confidence = det.confidence;
        
	det_msg.position.x = det.position.x;
	det_msg.position.y = det.position.y;
	det_msg.position.z = det.position.z;
	
	det_msg.dimensions.x = det.dimensions.x;
	det_msg.dimensions.y = det.dimensions.y;
	det_msg.dimensions.z = det.dimensions.z;

	det_msg.orientation.x = det.orientation.x;
	det_msg.orientation.y = det.orientation.y;
	det_msg.orientation.z = det.orientation.z;
	det_msg.orientation.w = det.orientation.w;

        msg.detections.push_back(det_msg);
    }
    
    // 发布检测结果
    mTomatoDetectionsPublisher.publish(msg);
}

} // namespace tomato_slam
#endif // TOMATO_SLAM_SYSTEM_INTEGRATOR_H
