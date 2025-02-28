#include <ros/ros.h>
#include "tomato_slam/integration/system_integrator.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "tomato_slam_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ROS_INFO("Starting Tomato SLAM system...");
    
    // 创建系统集成器
    tomato_slam::SystemIntegrator integrator(nh, pnh);
    
    // 初始化系统
    if (!integrator.initialize()) {
        ROS_ERROR("Failed to initialize system integrator");
        return 1;
    }
    
    // 运行系统
    integrator.run();
    
    ros::spin();
    
    ROS_INFO("Tomato SLAM system shutdown");
    
    return 0;
}
