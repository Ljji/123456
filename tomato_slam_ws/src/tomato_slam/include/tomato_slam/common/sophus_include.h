#pragma once

// 防止系统Sophus库和ORB_SLAM3的Sophus库冲突
#define SOPHUS_NO_EXPORTS

// 使用ORB_SLAM3的Sophus库
#include "/home/yw/ORB_SLAM3/Thirdparty/Sophus/sophus/se3.hpp"
