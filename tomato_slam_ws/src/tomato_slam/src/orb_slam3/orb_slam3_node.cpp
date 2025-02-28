#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "tomato_slam/orb_slam3/orb_slam3_interface.h"

class OrbSlam3Node {
public:
    OrbSlam3Node(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
        : mNH(nh), mPNH(pnh), mImageSubscriber(nullptr), mDepthSubscriber(nullptr), mSynchronizer(nullptr) {
        
        // 加载参数
        std::string vocFile, settingsFile;
        mPNH.param<std::string>("voc_file", vocFile, "");
        mPNH.param<std::string>("settings_file", settingsFile, "");
        mPNH.param<std::string>("camera_frame_id", mCameraFrameId, "camera_link");
        mPNH.param<std::string>("map_frame_id", mMapFrameId, "map");
        
        if (vocFile.empty() || settingsFile.empty()) {
            ROS_ERROR("Vocabulary file or settings file not specified");
            ros::shutdown();
            return;
        }
        
        // 创建发布者
        mPosePublisher = mNH.advertise<geometry_msgs::PoseStamped>("camera_pose", 1);
        mMapPointsPublisher = mNH.advertise<sensor_msgs::PointCloud2>("map_points", 1);
        
        // 初始化ORB-SLAM3
        ROS_INFO("Initializing ORB-SLAM3 with voc: %s, settings: %s", vocFile.c_str(), settingsFile.c_str());
        try {
            mSLAM = std::make_unique<tomato_slam::OrbSlam3Interface>(
                vocFile, settingsFile, ORB_SLAM3::System::RGBD);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to initialize ORB-SLAM3: %s", e.what());
            ros::shutdown();
            return;
        }
        
        // 设置RGB-D同步订阅
        mImageSubscriber = new message_filters::Subscriber<sensor_msgs::Image>(mNH, "camera/rgb/image_raw", 1);
        mDepthSubscriber = new message_filters::Subscriber<sensor_msgs::Image>(mNH, "camera/depth/image_raw", 1);
        
        mSynchronizer = new message_filters::Synchronizer<SyncPolicy>(
            SyncPolicy(10), *mImageSubscriber, *mDepthSubscriber);
        
        mSynchronizer->registerCallback(boost::bind(&OrbSlam3Node::rgbdCallback, this, _1, _2));
        
        ROS_INFO("ORB-SLAM3 node initialized");
    }
    
    ~OrbSlam3Node() {
        if (mSLAM) {
            mSLAM->shutdown();
        }
        
        delete mImageSubscriber;
        delete mDepthSubscriber;
        delete mSynchronizer;
    }
    
private:
    // ROS节点句柄
    ros::NodeHandle mNH;
    ros::NodeHandle mPNH;
    
    // 发布者
    ros::Publisher mPosePublisher;
    ros::Publisher mMapPointsPublisher;
    
    // TF广播器
    tf::TransformBroadcaster mTfBroadcaster;
    
    // 订阅者
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image>* mImageSubscriber;
    message_filters::Subscriber<sensor_msgs::Image>* mDepthSubscriber;
    message_filters::Synchronizer<SyncPolicy>* mSynchronizer;
    
    // ORB-SLAM3接口
    std::unique_ptr<tomato_slam::OrbSlam3Interface> mSLAM;
    
    // 坐标系ID
    std::string mCameraFrameId;
    std::string mMapFrameId;
    
    // 回调函数
    void rgbdCallback(const sensor_msgs::Image::ConstPtr& rgb_msg, 
                     const sensor_msgs::Image::ConstPtr& depth_msg) {
        try {
            // 转换RGB图像
            cv_bridge::CvImageConstPtr cv_rgb = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
            
            // 转换深度图像
            cv_bridge::CvImageConstPtr cv_depth = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
            
            // 将16位深度图转换为32位浮点数（毫米转米）
            cv::Mat depth_float;
            cv_depth->image.convertTo(depth_float, CV_32F, 1.0/1000.0);
            
            // 获取时间戳
            double timestamp = rgb_msg->header.stamp.toSec();
            
            // 处理RGBD图像
            Eigen::Matrix4f pose;
            bool success = mSLAM->processRGBD(cv_rgb->image, depth_float, timestamp, pose);
            
            if (success) {
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
    
    // 发布位姿
    void publishPose(const Eigen::Matrix4f& pose, const ros::Time& timestamp) {
        // 创建位姿消息
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = timestamp;
        pose_msg.header.frame_id = mMapFrameId;
        
        // 从位姿矩阵提取平移
        pose_msg.pose.position.x = pose(0, 3);
        pose_msg.pose.position.y = pose(1, 3);
        pose_msg.pose.position.z = pose(2, 3);
        
        // 从位姿矩阵提取旋转四元数
        Eigen::Matrix3f rot = pose.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rot);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        
        // 发布位姿
        mPosePublisher.publish(pose_msg);
        
        // 发布TF变换
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z));
            
        transform.setRotation(tf::Quaternion(
            q.x(), q.y(), q.z(), q.w()));
            
        mTfBroadcaster.sendTransform(tf::StampedTransform(
            transform, timestamp, mMapFrameId, mCameraFrameId));
    }
    
    // 发布地图点云
    void publishMapPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& mapPoints,
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "orb_slam3_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    OrbSlam3Node node(nh, pnh);
    
    ros::spin();
    
    return 0;
}
