#include <iostream>
#include <fstream>
#include <vector>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

// Logger for TensorRT
class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        // 根据严重性打印消息
        switch (severity) {
            case Severity::kINTERNAL_ERROR:
                std::cerr << "INTERNAL_ERROR: " << msg << std::endl;
                break;
            case Severity::kERROR:
                std::cerr << "ERROR: " << msg << std::endl;
                break;
            case Severity::kWARNING:
                std::cerr << "WARNING: " << msg << std::endl;
                break;
            case Severity::kINFO:
                std::cout << "INFO: " << msg << std::endl;
                break;
            default:
                std::cout << "DEBUG: " << msg << std::endl;
                break;
        }
    }
} gLogger;

// 构建TensorRT引擎
nvinfer1::ICudaEngine* buildEngine(const std::string& onnxModelPath, nvinfer1::IBuilder* builder) {
    // 创建网络
    const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    nvinfer1::INetworkDefinition* network = builder->createNetworkV2(explicitBatch);
    
    // 创建ONNX解析器
    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, gLogger);
    
    // 解析ONNX模型
    bool parsed = parser->parseFromFile(onnxModelPath.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING));
    if (!parsed) {
        std::cerr << "Failed to parse ONNX model: " << onnxModelPath << std::endl;
        return nullptr;
    }
    
    // 打印网络输入输出信息
    std::cout << "Network has " << network->getNbInputs() << " inputs:" << std::endl;
    for (int i = 0; i < network->getNbInputs(); i++) {
        auto input = network->getInput(i);
        auto dims = input->getDimensions();
        std::cout << "  Input " << i << ": " << input->getName() << ", dims: ";
        for (int j = 0; j < dims.nbDims; j++) {
            std::cout << dims.d[j] << " ";
        }
        std::cout << std::endl;
    }
    
    std::cout << "Network has " << network->getNbOutputs() << " outputs:" << std::endl;
    for (int i = 0; i < network->getNbOutputs(); i++) {
        auto output = network->getOutput(i);
        auto dims = output->getDimensions();
        std::cout << "  Output " << i << ": " << output->getName() << ", dims: ";
        for (int j = 0; j < dims.nbDims; j++) {
            std::cout << dims.d[j] << " ";
        }
        std::cout << std::endl;
    }
    
    // 创建构建配置
    nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
    
    // 设置最大工作空间大小
    config->setMaxWorkspaceSize(1ULL << 30); // 1GB
    
    // 设置精度模式
    std::cout << "Setting FP16 mode..." << std::endl;
    if (builder->platformHasFastFp16()) {
        std::cout << "Platform supports FP16, enabling..." << std::endl;
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
    } else {
        std::cout << "Platform does not support FP16, using FP32..." << std::endl;
    }
    
    // 构建引擎
    std::cout << "Building TensorRT engine. This may take a while..." << std::endl;
    nvinfer1::ICudaEngine* engine = builder->buildEngineWithConfig(*network, *config);
    
    if (!engine) {
        std::cerr << "Failed to build TensorRT engine." << std::endl;
        return nullptr;
    }
    
    std::cout << "Engine built successfully!" << std::endl;
    
    // 清理资源
    parser->destroy();
    network->destroy();
    config->destroy();
    
    return engine;
}

// 保存引擎到文件
bool saveEngine(nvinfer1::ICudaEngine* engine, const std::string& fileName) {
    if (!engine) {
        std::cerr << "Invalid engine." << std::endl;
        return false;
    }
    
    // 序列化引擎
    nvinfer1::IHostMemory* serializedEngine = engine->serialize();
    if (!serializedEngine) {
        std::cerr << "Failed to serialize engine." << std::endl;
        return false;
    }
    
    // 写入文件
    std::ofstream engineFile(fileName, std::ios::binary);
    if (!engineFile) {
        std::cerr << "Failed to open engine file for writing: " << fileName << std::endl;
        serializedEngine->destroy();
        return false;
    }
    
    engineFile.write(static_cast<const char*>(serializedEngine->data()), serializedEngine->size());
    if (engineFile.fail()) {
        std::cerr << "Failed to write engine to file: " << fileName << std::endl;
        serializedEngine->destroy();
        return false;
    }
    
    // 清理资源
    serializedEngine->destroy();
    
    std::cout << "Engine saved to: " << fileName << std::endl;
    return true;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <path_to_onnx_model> <output_engine_file>" << std::endl;
        return 1;
    }
    
    std::string onnxModelPath = argv[1];
    std::string engineFilePath = argv[2];
    
    // 初始化CUDA
    cudaSetDevice(0);
    
    // 创建TensorRT builder
    nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(gLogger);
    if (!builder) {
        std::cerr << "Failed to create TensorRT builder." << std::endl;
        return 1;
    }
    
    // 构建引擎
    nvinfer1::ICudaEngine* engine = buildEngine(onnxModelPath, builder);
    if (!engine) {
        std::cerr << "Failed to build TensorRT engine." << std::endl;
        builder->destroy();
        return 1;
    }
    
    // 保存引擎
    bool saved = saveEngine(engine, engineFilePath);
    
    // 清理资源
    engine->destroy();
    builder->destroy();
    
    return saved ? 0 : 1;
}
