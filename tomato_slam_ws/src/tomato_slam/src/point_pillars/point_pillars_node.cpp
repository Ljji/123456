#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Dense>

#include "tomato_slam/TomatoDetections.h"
#include "tomato_slam/point_pillars/point_pillars_detector.h"

class PointPillarsNode {
public:
    PointPillarsNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
        : mNH(nh), mPNH(pnh) {
        
        // 加载参数
        std::string engineFile;
        float nmsThreshold, scoreThreshold;
        
        mPNH.param<std::string>("engine_file", engineFile, "");
        mPNH.param<float>("nms_threshold", nmsThreshold, 0.5f);
        mPNH.param<float>("score_threshold", scoreThreshold, 0.3f);
        mPNH.param<std::string>("camera_frame_id", mCameraFrameId, "camera_link");
        mPNH.param<std::string>("map_frame_id", mMapFrameId, "map");
        mPNH.param<double>("detection_rate", mDetectionRate, 5.0);
        mPNH.param<bool>("visualize_detections", mVisualizeDetections, true);
        
        if (engineFile.empty()) {
            ROS_ERROR("Engine file not specified");
            ros::shutdown();
            return;
        }
        
        // 创建发布者
        mTomatoDetectionsPublisher = mNH.advertise<tomato_slam::TomatoDetections>("tomato_detections", 1);
        
        if (mVisualizeDetections) {
            mMarkerPublisher = mNH.advertise<visualization_msgs::MarkerArray>("tomato_markers", 1);
        }
        
        // 订阅点云和相机位姿
        mPointCloudSubscriber = mNH.subscribe("camera/depth/points", 1, &PointPillarsNode::pointCloudCallback, this);
        mPoseSubscriber = mNH.subscribe("camera_pose", 1, &PointPillarsNode::poseCallback, this);
        
        // 初始化PointPillars检测器
        ROS_INFO("Initializing PointPillars detector with engine: %s", engineFile.c_str());
        try {
            mDetector = std::make_unique<tomato_slam::PointPillarsDetector>(
                engineFile, nmsThreshold, scoreThreshold);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to initialize PointPillars detector: %s", e.what());
            ros::shutdown();
            return;
        }
        
        // 初始化位姿为单位矩阵
        mLatestPose = Eigen::Matrix4f::Identity();
        mHasPose = false;
        
        // 启动定时器进行定期检测
        mDetectionTimer = mNH.createTimer(
            ros::Duration(1.0 / mDetectionRate), &PointPillarsNode::detectionTimerCallback, this);
        
        ROS_INFO("PointPillars node initialized");
    }
    
private:
    // ROS节点句柄
    ros::NodeHandle mNH;
    ros::NodeHandle mPNH;
    
    // 发布者
    ros::Publisher mTomatoDetectionsPublisher;
    ros::Publisher mMarkerPublisher;
    
    // 订阅者
    ros::Subscriber mPointCloudSubscriber;
    ros::Subscriber mPoseSubscriber;
    
    // 定时器
    ros::Timer mDetectionTimer;
    
    // PointPillars检测器
    std::unique_ptr<tomato_slam::PointPillarsDetector> mDetector;
    
    // 最新点云和位姿
    pcl::PointCloud<pcl::PointXYZ>::Ptr mLatestPointCloud;
    Eigen::Matrix4f mLatestPose;
    bool mHasPose;
    
    // 坐标系ID
    std::string mCameraFrameId;
    std::string mMapFrameId;
    
    // 参数
    double mDetectionRate;
    bool mVisualizeDetections;
    
    // 互斥锁
    std::mutex mPointCloudMutex;
    std::mutex mPoseMutex;
    
    // 点云回调
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        
        std::lock_guard<std::mutex> lock(mPointCloudMutex);
        mLatestPointCloud = cloud;
    }
    
    // 位姿回调
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 将ROS位姿转换为Eigen矩阵
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        
        // 设置平移
        pose(0, 3) = msg->pose.position.x;
        pose(1, 3) = msg->pose.position.y;
        pose(2, 3) = msg->pose.position.z;
        
        // 设置旋转
        Eigen::Quaternionf q(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);
        pose.block<3, 3>(0, 0) = q.toRotationMatrix();
        
        std::lock_guard<std::mutex> lock(mPoseMutex);
        mLatestPose = pose;
        mHasPose = true;
    }
    
    // 定时检测回调
    void detectionTimerCallback(const ros::TimerEvent& event) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        Eigen::Matrix4f pose;
        bool hasPose;
        
        // 获取最新点云和位姿
        {
            std::lock_guard<std::mutex> lock1(mPointCloudMutex);
            std::lock_guard<std::mutex> lock2(mPoseMutex);
            
            if (!mLatestPointCloud || mLatestPointCloud->empty() || !mHasPose) {
                return;
            }
            
            cloud = mLatestPointCloud;
            pose = mLatestPose;
            hasPose = mHasPose;
        }
        
        if (!hasPose) {
            ROS_WARN_THROTTLE(1.0, "No camera pose available for transformation");
            return;
        }
        
        // 检测番茄
        std::vector<tomato_slam::TomatoDetection> detections;
        bool success = mDetector->detectTomatoes(cloud, detections);
        
        if (success) {
            // 将检测结果从相机坐标系转换到世界坐标系
            mDetector->transformDetections(detections, pose);
            
            // 发布检测结果
            publishTomatoDetections(detections, event.current_real);
            
            // 可视化检测结果
            if (mVisualizeDetections) {
                publishVisualization(detections, event.current_real);
            }
        }
    }
    
    // 发布番茄检测结果
    void publishTomatoDetections(const std::vector<tomato_slam::TomatoDetection>& detections,
                               const ros::Time& timestamp) {
        tomato_slam::TomatoDetections msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = mMapFrameId;
        
        for (const auto& det : detections) {
            tomato_slam::TomatoDetection det_msg;
            det_msg.id = det.id;
            det_msg.class_name = det.class_name;
            det_msg.confidence = det.confidence;
            
            // 设置位置
            det_msg.position.x = det.position.x();
            det_msg.position.y = det.position.y();
            det_msg.position.z = det.position.z();
            
            // 设置尺寸
            det_msg.dimensions.x = det.dimensions.x();
            det_msg.dimensions.y = det.dimensions.y();
            det_msg.dimensions.z = det.dimensions.z();
            
            // 设置方向
            det_msg.orientation.x = det.orientation.x();
            det_msg.orientation.y = det.orientation.y();
            det_msg.orientation.z = det.orientation.z();
            det_msg.orientation.w = det.orientation.w();
            
            msg.detections.push_back(det_msg);
        }
        
        mTomatoDetectionsPublisher.publish(msg);
    }
    
    // 发布可视化标记
    void publishVisualization(const std::vector<tomato_slam::TomatoDetection>& detections,
                             const ros::Time& timestamp) {
        visualization_msgs::MarkerArray marker_array;
        
        // 创建一个删除之前的所有标记的标记
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = mMapFrameId;
        clear_marker.header.stamp = timestamp;
        clear_marker.ns = "tomatoes";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        // 为每个检测结果创建标记
        for (size_t i = 0; i < detections.size(); ++i) {
            const auto& det = detections[i];
            
            // 创建球体标记表示番茄
            visualization_msgs::Marker marker;
            marker.header.frame_id = mMapFrameId;
            marker.header.stamp = timestamp;
            marker.ns = "tomatoes";
            marker.id = i + 1;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            
            // 设置位置
            marker.pose.position.x = det.position.x();
            marker.pose.position.y = det.position.y();
            marker.pose.position.z = det.position.z();
            
            // 设置方向
            marker.pose.orientation.x = det.orientation.x();
            marker.pose.orientation.y = det.orientation.y();
            marker.pose.orientation.z = det.orientation.z();
            marker.pose.orientation.w = det.orientation.w();
            
            // 设置尺寸 (取平均半径)
            float radius = (det.dimensions.x() + det.dimensions.y() + det.dimensions.z()) / 6.0f;
            marker.scale.x = radius * 2;
            marker.scale.y = radius * 2;
            marker.scale.z = radius * 2;
            
            // 设置颜色 (红色表示番茄)
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.7;
            
            // 设置持久时间
            marker.lifetime = ros::Duration(1.0 / mDetectionRate * 2.0);
            
            marker_array.markers.push_back(marker);
            
            // 创建文本标记显示置信度
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = mMapFrameId;
            text_marker.header.stamp = timestamp;
            text_marker.ns = "tomato_labels";
            text_marker.id = i + 1;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            
            // 设置位置 (稍微在番茄上方)
            text_marker.pose.position.x = det.position.x();
            text_marker.pose.position.y = det.position.y();
            text_marker.pose.position.z = det.position.z() + radius * 2;
            
            // 设置文本
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << det.confidence * 100.0 << "%";
            text_marker.text = ss.str();
            
            // 设置尺寸
            text_marker.scale.z = 0.1;
            
            // 设置颜色 (白色)
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            // 设置持久时间
            text_marker.lifetime = ros::Duration(1.0 / mDetectionRate * 2.0);
            
            marker_array.markers.push_back(text_marker);
        }
        
        mMarkerPublisher.publish(marker_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_pillars_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    PointPillarsNode node(nh, pnh);
    
    ros::spin();
    
    return 0;
}
