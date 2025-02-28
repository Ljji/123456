// ~/tomato_slam_ws/src/tomato_slam/src/point_pillars/tensorrt/tensorrt_inference.cpp
#include <tomato_slam/point_pillars/tensorrt/tensorrt_inference.h>

namespace tomato_slam {

TensorRTInference::TensorRTInference(const std::string& engine_path) {
  // TensorRT 8的初始化
  logger_ = std::make_unique<Logger>();
  runtime_ = std::unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(*logger_));
  
  // 加载引擎文件
  std::ifstream engine_file(engine_path, std::ios::binary);
  if (!engine_file) {
    throw std::runtime_error("无法打开TensorRT引擎文件: " + engine_path);
  }

  engine_file.seekg(0, std::ios::end);
  size_t size = engine_file.tellg();
  engine_file.seekg(0, std::ios::beg);

  std::vector<char> engine_data(size);
  engine_file.read(engine_data.data(), size);
  
  // 创建执行上下文
  engine_ = std::unique_ptr<nvinfer1::ICudaEngine>(
      runtime_->deserializeCudaEngine(engine_data.data(), size));
  if (!engine_) {
    throw std::runtime_error("无法反序列化TensorRT引擎");
  }
  
  context_ = std::unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  if (!context_) {
    throw std::runtime_error("无法创建TensorRT执行上下文");
  }
  
  // 设置输入输出缓冲区
  setupBuffers();
}

TensorRTInference::~TensorRTInference() {
  // 释放CUDA内存
  for (auto& buffer : device_buffers_) {
    cudaFree(buffer);
  }
  for (auto& buffer : host_buffers_) {
    cudaFreeHost(buffer);
  }
}

void TensorRTInference::setupBuffers() {
  // 分配输入输出缓冲区
  int num_bindings = engine_->getNbBindings();
  device_buffers_.resize(num_bindings);
  host_buffers_.resize(num_bindings);
  
  // 获取所有绑定尺寸
  for (int i = 0; i < num_bindings; i++) {
    nvinfer1::Dims dims = engine_->getBindingDimensions(i);
    size_t size = 1;
    for (int j = 0; j < dims.nbDims; j++) {
      size *= dims.d[j];
    }
    
    size_t element_size = sizeof(float);  // 假设所有输入/输出都是float
    size *= element_size;
    
    // 分配CUDA设备和主机内存
    cudaMalloc(&device_buffers_[i], size);
    cudaMallocHost(&host_buffers_[i], size);
    
    // 存储绑定信息
    std::string name = engine_->getBindingName(i);
    binding_map_[name] = i;
    binding_sizes_[name] = size;
    
    if (engine_->bindingIsInput(i)) {
      input_names_.push_back(name);
    } else {
      output_names_.push_back(name);
    }
  }
}

void TensorRTInference::infer() {
  // 将主机内存数据复制到设备内存
  for (int i = 0; i < input_names_.size(); i++) {
    int binding_idx = binding_map_[input_names_[i]];
    size_t size = binding_sizes_[input_names_[i]];
    cudaMemcpy(device_buffers_[binding_idx], host_buffers_[binding_idx], 
              size, cudaMemcpyHostToDevice);
  }
  
  // 执行推理
  // TensorRT 8中的执行方法有所变化
  void* bindings[device_buffers_.size()];
  for (int i = 0; i < device_buffers_.size(); i++) {
    bindings[i] = device_buffers_[i];
  }
  
  // 在TensorRT 8中，enqueue被enqueueV2替代
  bool status = context_->executeV2(bindings);
  if (!status) {
    throw std::runtime_error("TensorRT推理失败");
  }
  
  // 将结果从设备内存复制回主机内存
  for (int i = 0; i < output_names_.size(); i++) {
    int binding_idx = binding_map_[output_names_[i]];
    size_t size = binding_sizes_[output_names_[i]];
    cudaMemcpy(host_buffers_[binding_idx], device_buffers_[binding_idx], 
              size, cudaMemcpyDeviceToHost);
  }
}

// 获取指向特定输入/输出缓冲区的指针
void* TensorRTInference::getBuffer(const std::string& name) {
  if (binding_map_.find(name) == binding_map_.end()) {
    throw std::runtime_error("绑定名称不存在: " + name);
  }
  int idx = binding_map_[name];
  return host_buffers_[idx];
}

size_t TensorRTInference::getBufferSize(const std::string& name) {
  if (binding_sizes_.find(name) == binding_sizes_.end()) {
    throw std::runtime_error("绑定名称不存在: " + name);
  }
  return binding_sizes_[name];
}

} // namespace tomato_slam
