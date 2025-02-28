#include "tomato_slam/orb_slam3/orb_slam3_interface.h"
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

namespace tomato_slam {
OrbSlam3Interface::OrbSlam3Interface(const std::string& vocFile, const std::string& settingsFile,
                                    ORB_SLAM3::System::eSensor sensorType)
    : mIsTracking(false) {
    // 使用第二个构造函数 (使用 std::__cxx11::string 类型的第6个参数)
    std::string seq_param;  // 创建一个 std::__cxx11::string 类型的变量
    const std::string topic_param = "";  // 创建一个 const string& 类型的变量
    
    mpSLAM = new ORB_SLAM3::System(
        vocFile,                // 第1个参数
        settingsFile,           // 第2个参数
        sensorType,             // 第3个参数
        true,                   // 第4个参数
        0,                      // 第5个参数
        seq_param,              // 第6个参数，使用 std::__cxx11::string 类型
        topic_param             // 第7个参数，使用 const string& 类型
    );
    
    mCurrentPose = Eigen::Matrix4f::Identity();
    ROS_INFO("ORB-SLAM3 interface initialized");
}

OrbSlam3Interface::~OrbSlam3Interface() {
    shutdown();
}

bool OrbSlam3Interface::processRGBD(const cv::Mat& im, const cv::Mat& depthmap, double timestamp, Eigen::Matrix4f& pose) {
    if (im.empty() || depthmap.empty()) {
        ROS_WARN("Empty RGB or depth image");
        return false;
    }
    
    // 使用ORB-SLAM3的TrackRGBD函数
    Sophus::SE3f track_result = mpSLAM->TrackRGBD(im, depthmap, timestamp);
    
    // 处理跟踪失败的情况
    if (track_result.matrix3x4().allFinite() == false) {
        mIsTracking = false;
        return false;
    }
    
    mIsTracking = true;
    
    // 将Sophus SE3转换为Eigen Matrix4f
    Eigen::Matrix4f eigenTcw = Eigen::Matrix4f::Identity();
    eigenTcw.block<3,3>(0,0) = track_result.rotationMatrix().cast<float>();
    eigenTcw.block<3,1>(0,3) = track_result.translation().cast<float>();
    
    // 相机到世界坐标系的变换
    mCurrentPose = eigenTcw.inverse();
    pose = mCurrentPose;
    
    return true;
}

bool OrbSlam3Interface::processMono(const cv::Mat& im, double timestamp, Eigen::Matrix4f& pose) {
    if (im.empty()) {
        ROS_WARN("Empty mono image");
        return false;
    }
    
    // 使用ORB-SLAM3的TrackMonocular函数
    Sophus::SE3f track_result = mpSLAM->TrackMonocular(im, timestamp);
    
    // 处理跟踪失败的情况
    if (track_result.matrix3x4().allFinite() == false) {
        mIsTracking = false;
        return false;
    }
    
    mIsTracking = true;
    
    // 将Sophus SE3转换为Eigen Matrix4f
    Eigen::Matrix4f eigenTcw = Eigen::Matrix4f::Identity();
    eigenTcw.block<3,3>(0,0) = track_result.rotationMatrix().cast<float>();
    eigenTcw.block<3,1>(0,3) = track_result.translation().cast<float>();
    
    // 相机到世界坐标系的变换
    mCurrentPose = eigenTcw.inverse();
    pose = mCurrentPose;
    
    return true;
}

bool OrbSlam3Interface::processStereo(const cv::Mat& imLeft, const cv::Mat& imRight, double timestamp, Eigen::Matrix4f& pose) {
    if (imLeft.empty() || imRight.empty()) {
        ROS_WARN("Empty stereo images");
        return false;
    }
    
    // 使用ORB-SLAM3的TrackStereo函数
    Sophus::SE3f track_result = mpSLAM->TrackStereo(imLeft, imRight, timestamp);
    
    // 处理跟踪失败的情况
    if (track_result.matrix3x4().allFinite() == false) {
        mIsTracking = false;
        return false;
    }
    
    mIsTracking = true;
    
    // 将Sophus SE3转换为Eigen Matrix4f
    Eigen::Matrix4f eigenTcw = Eigen::Matrix4f::Identity();
    eigenTcw.block<3,3>(0,0) = track_result.rotationMatrix().cast<float>();
    eigenTcw.block<3,1>(0,3) = track_result.translation().cast<float>();
    
    // 相机到世界坐标系的变换
    mCurrentPose = eigenTcw.inverse();
    pose = mCurrentPose;
    
    return true;
}

void OrbSlam3Interface::getCurrentMapPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& mapPointCloud) {
    if (!mpSLAM) return;
    
    // 获取所有地图点
    auto mapPoints = mpSLAM->GetTrackedMapPoints();
    
    mapPointCloud->clear();
    
    for (auto& pMP : mapPoints) {
        if (!pMP || pMP->isBad())
            continue;
            
        // 获取地图点的3D位置
        Eigen::Vector3f pos = pMP->GetWorldPos();
        
        // 将地图点添加到点云中
        pcl::PointXYZRGB pt;
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        
        // 根据观测次数设置颜色
        float ratio = std::min(1.0f, static_cast<float>(pMP->Observations()) / 10.0f);
        pt.r = static_cast<uint8_t>(255 * (1.0f - ratio));
        pt.g = static_cast<uint8_t>(255 * ratio);
        pt.b = 0;
        
        mapPointCloud->push_back(pt);
    }
}

void OrbSlam3Interface::shutdown() {
    if (mpSLAM) {
        mpSLAM->Shutdown();
        delete mpSLAM;
        mpSLAM = nullptr;
    }
}

bool OrbSlam3Interface::isTracking() const {
    std::unique_lock<std::mutex> lock(mMutex);
    return mIsTracking;
}

Eigen::Matrix4f OrbSlam3Interface::getCurrentPose() const {
    std::unique_lock<std::mutex> lock(mMutex);
    return mCurrentPose;
}

} // namespace tomato_slam
