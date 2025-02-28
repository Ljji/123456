#pragma once
#include <Eigen/Core>
#include <string>

namespace tomato_slam {

struct BoundingBox {
    Eigen::Vector3f position;  // 中心位置
    Eigen::Vector3f dimensions; // 长宽高
    float radius;              // 半径(如适用)
    float confidence;          // 置信度
    int class_id;              // 类别ID
    std::string class_name;    // 类别名称
};

} // namespace tomato_slam
