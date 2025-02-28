cmake_minimum_required(VERSION 3.0.2)
project(tomato_slam)

# 编译选项
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_ros
  cv_bridge
  image_transport
  tf2
  tf2_ros
  message_generation
)

# 查找 PCL
find_package(PCL 1.8 REQUIRED)

# 查找 OpenCV
find_package(OpenCV 4.0 REQUIRED)

# 查找 CUDA
find_package(CUDA REQUIRED)

# 查找 Eigen
find_package(Eigen3 REQUIRED)

# 添加自定义消息
add_message_files(
  FILES
  TomatoDetection.msg
  TomatoDetections.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 包含目录
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs visualization_msgs pcl_ros cv_bridge image_transport tf2 tf2_ros message_runtime
  DEPENDS PCL OpenCV EIGEN3 CUDA
)

# ORB-SLAM3 目录
set(ORB_SLAM3_DIR $ENV{HOME}/ORB_SLAM3)
# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels  # 添加这一行
  ${ORB_SLAM3_DIR}/Thirdparty
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
)

# 链接目录
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${ORB_SLAM3_DIR}/lib
)

# CUDA 编译选项
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -O3 -gencode arch=compute_61,code=sm_61)

# 编译 CUDA 文件
cuda_add_library(pointpillars_cuda
  src/point_pillars/tensorrt/preprocess_kernels.cu
  src/point_pillars/tensorrt/postprocess_kernels.cu
)

# 主库
add_library(${PROJECT_NAME}
  src/orb_slam3/orb_slam3_interface.cpp
  src/point_pillars/point_pillars_detector.cpp
  src/point_pillars/tensorrt/tensorrt_inference.cpp
  src/integration/system_integrator.cpp
  src/visualization/detection_visualizer.cpp
)

# 链接
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  pointpillars_cuda
  -lORB_SLAM3
  -ldbow2
  -lDBoW2
  -lg2o
  -lnvinfer
  -lnvinfer_plugin
)

# 各节点
add_executable(orb_slam3_node src/orb_slam3/orb_slam3_node.cpp)
target_link_libraries(orb_slam3_node ${PROJECT_NAME})

add_executable(point_pillars_node src/point_pillars/point_pillars_node.cpp)
target_link_libraries(point_pillars_node ${PROJECT_NAME})

add_executable(tomato_slam_node src/integration/tomato_slam_node.cpp)
target_link_libraries(tomato_slam_node ${PROJECT_NAME})

add_executable(build_engine src/point_pillars/tensorrt/build_engine.cpp)
target_link_libraries(build_engine ${PROJECT_NAME})
