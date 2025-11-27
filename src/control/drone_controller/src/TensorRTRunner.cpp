/**
 * @file    TensorRTRunner.cpp
 * @brief   TensorRT 推理封装（单输入/单输出 + 精简批量接口，含非连续批量支持）
 * 
 * @author  -
 * @date    2025-08-10
 * @version v1.6
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "TensorRTRunner.hpp"

#include <algorithm>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>

namespace trt
{

/************************************************************************************************************************
 * @brief   TensorRT 日志输出回调（彩色）
 * @details 仅输出 WARNING 及以上等级日志，避免冗余信息刷屏。若需要更详细日志，可放宽等级。
 * @param   severity    日志等级
 * @param   msg         日志内容
 ***********************************************************************************************************************/
void Logger::log(Severity severity, const char* msg) noexcept
{
    const char* color = "";
    const char* reset = "\033[0m";
    const char* level = "";

    switch (severity)
    {
        case Severity::kINTERNAL_ERROR: color = "\033[1;31m"; level = "[FATAL]";   break;
        case Severity::kERROR:          color = "\033[31m";   level = "[ERROR]";   break;
        case Severity::kWARNING:        color = "\033[33m";   level = "[WARNING]"; break;
        case Severity::kINFO:           color = "\033[32m";   level = "[INFO]";    break;
        case Severity::kVERBOSE:        color = "\033[36m";   level = "[VERBOSE]"; break;
        default:                        color = "\033[0m";    level = "[UNKNOWN]"; break;
    }

    std::cout << color << level << " [TRT] " << msg << reset << "\n";
}

/************************************************************************************************************************
 * @brief   CUDA 错误检查
 * @details 将 CUDA API 返回码转换为可读信息并输出源位置。当前策略为直接终止进程；
 *          若项目更偏向异常处理，可替换为 throw std::runtime_error(...)。
 * @param   e       CUDA API 返回错误码
 * @param   file    出错源文件名
 * @param   line    出错行号
 ***********************************************************************************************************************/
void checkCuda(cudaError_t e, const char* file, int line)
{
    if (e != cudaSuccess)
    {
        std::cerr << "CUDA error: " << cudaGetErrorString(e)
                  << " at " << file << ":" << line << "\n";
        std::exit(1);
    }
}

/************************************************************************************************************************
 * @brief   从文件读取二进制数据
 * @details 以二进制形式读取 .plan 等文件，并返回完整内容。
 * @param   path    文件路径
 * @return  文件内容的字节数组
 * @throw   std::runtime_error  打开或读取失败
 ***********************************************************************************************************************/
std::vector<char> TensorRTRunner::readFile(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
    {
        throw std::runtime_error("Failed to open " + path);
    }

    const std::streamsize size = f.tellg();
    std::vector<char> buf(static_cast<size_t>(size));
    f.seekg(0, std::ios::beg);
    f.read(buf.data(), size);
    return buf;
}

/************************************************************************************************************************
 * @brief   构造函数：加载并反序列化 TensorRT engine
 * @details 流程：
 *          1) 创建 IRuntime
 *          2) 读取 .plan 文件并反序列化为 ICudaEngine
 *          3) 创建 IExecutionContext
 *          4) 绑定与数据类型校验（仅支持 1 入 1 出，FP32）
 *          5) 创建 CUDA Stream
 * @param   enginePath  序列化的 .plan 文件路径
 * @throw   std::runtime_error  反序列化失败 / 资源创建失败
 ***********************************************************************************************************************/
TensorRTRunner::TensorRTRunner(const std::string& enginePath)
    : runtime_(nvinfer1::createInferRuntime(gLogger_))
{
    if (!runtime_)
    {
        throw std::runtime_error("createInferRuntime failed");
    }

    const auto engineData = readFile(enginePath);

    engine_.reset(runtime_->deserializeCudaEngine(engineData.data(), engineData.size()));
    if (!engine_)
    {
        throw std::runtime_error("deserializeCudaEngine failed");
    }

    context_.reset(engine_->createExecutionContext());
    if (!context_)
    {
        throw std::runtime_error("createExecutionContext failed");
    }

    validateBindingsAndTypes();

    TRT_CHECK_CUDA(cudaStreamCreate(&stream_));
}

/************************************************************************************************************************
 * @brief   析构：释放 CUDA 资源
 * @details 释放顺序：设备缓冲区（输入/输出） -> CUDA Stream。智能指针会自动释放 TRT 对象。
 ***********************************************************************************************************************/
TensorRTRunner::~TensorRTRunner()
{
    allocateOrReuseDevice_(/*bytesChanged=*/true); // 将内部指针释放为 nullptr
    if (stream_)
    {
        cudaStreamDestroy(stream_);
        stream_ = nullptr;
    }
}

/************************************************************************************************************************
 * @brief   一次性校验：1 入 1 出 + FP32 数据类型
 * @throw   std::runtime_error  绑定数量/类型不符合要求
 ***********************************************************************************************************************/
void TensorRTRunner::validateBindingsAndTypes()
{
    const int nb = engine_->getNbBindings();
    if (nb != 2)
    {
        throw std::runtime_error("This runner expects exactly 1 input and 1 output (got " + std::to_string(nb) + ").");
    }

    for (int i = 0; i < nb; ++i)
    {
        if (engine_->bindingIsInput(i))  inputIndex_  = i;
        else                             outputIndex_ = i;
    }

    if (inputIndex_ < 0 || outputIndex_ < 0)
    {
        throw std::runtime_error("Failed to locate input/output bindings.");
    }

    const auto inType  = engine_->getBindingDataType(inputIndex_);
    const auto outType = engine_->getBindingDataType(outputIndex_);
    if (inType != nvinfer1::DataType::kFLOAT || outType != nvinfer1::DataType::kFLOAT)
    {
        throw std::runtime_error("Only FP32 is supported in this runner (binding types mismatch).");
    }
}

/************************************************************************************************************************
 * @brief   Profile 范围校验（仅对动态 profile 有意义）
 * @param   d   期望绑定维度
 * @throw   std::runtime_error  任一维度越界
 ***********************************************************************************************************************/
void TensorRTRunner::validateWithinProfile(const nvinfer1::Dims& d)
{
    const int prof = context_->getOptimizationProfile();
    if (prof < 0) return; // 兼容旧接口

    auto minD = engine_->getProfileDimensions(inputIndex_, prof, nvinfer1::OptProfileSelector::kMIN);
    auto maxD = engine_->getProfileDimensions(inputIndex_, prof, nvinfer1::OptProfileSelector::kMAX);

    if (minD.nbDims <= 0 || maxD.nbDims <= 0) return;
    if (minD.nbDims != d.nbDims || maxD.nbDims != d.nbDims)  return;

    bool dynamic = false;
    for (int i = 0; i < minD.nbDims; ++i)
    {
        if (minD.d[i] != maxD.d[i]) { dynamic = true; break; }
    }
    if (!dynamic) return; // 静态 profile

    for (int i = 0; i < d.nbDims; ++i)
    {
        const int mn = minD.d[i];
        const int mx = maxD.d[i];
        if (mn >= 0 && d.d[i] < mn)
        {
            throw std::runtime_error("setInputShape: dim(" + std::to_string(i) + ")=" + std::to_string(d.d[i]) +
                                     " < profile min=" + std::to_string(mn));
        }
        if (mx >= 0 && d.d[i] > mx)
        {
            throw std::runtime_error("setInputShape: dim(" + std::to_string(i) + ")=" + std::to_string(d.d[i]) +
                                     " > profile max=" + std::to_string(mx));
        }
    }
}

/************************************************************************************************************************
 * @brief   应用输入维度（统一入口）
 * @details 维度构造 → Profile 校验 → setBinding → 校验 in/out → 若尺寸变化才重分配
 * @param   sampleDims  样本维（不含 batch）
 * @param   batchSize   批大小（>0）
 * @throw   std::runtime_error  越界/绑定失败/维度未指定
 ***********************************************************************************************************************/
void TensorRTRunner::applyInputDims_(const std::vector<int>& sampleDims, int batchSize)
{
    if (batchSize <= 0)
    {
        throw std::runtime_error("batchSize must be > 0");
    }

    nvinfer1::Dims want{};                      // 零初始化，避免脏值
    want.nbDims = static_cast<int>(sampleDims.size() + 1);
    want.d[0]   = batchSize;
    for (int i = 0; i < static_cast<int>(sampleDims.size()); ++i)
    {
        want.d[i + 1] = sampleDims[i];
    }

    validateWithinProfile(want);

    if (!context_->setBindingDimensions(inputIndex_, want))
    {
        throw std::runtime_error("setBindingDimensions(sampleDims,batch) failed");
    }

    const auto inAfter  = context_->getBindingDimensions(inputIndex_);
    const auto outAfter = context_->getBindingDimensions(outputIndex_);
    if (!isDimsSpecified(inAfter))
    {
        throw std::runtime_error("applyInputDims_: input dims unspecified after binding");
    }
    if (!isDimsSpecified(outAfter))
    {
        throw std::runtime_error("applyInputDims_: output dims unspecified after binding");
    }

    const size_t newInputCount  = volume(inAfter);
    const size_t newOutputCount = volume(outAfter);
    const size_t newInputBytes  = newInputCount  * sizeof(float);
    const size_t newOutputBytes = newOutputCount * sizeof(float);

    currentBatch_             = std::max(1, inAfter.d[0]);
    inputElemsPerSample_      = newInputCount  / static_cast<size_t>(currentBatch_);
    outputElemsPerSample_     = newOutputCount / static_cast<size_t>(currentBatch_);

    const bool bytesChanged =
        (!allocated_) || (newInputBytes != inputBytes_) || (newOutputBytes != outputBytes_);

    inputCount_  = newInputCount;
    outputCount_ = newOutputCount;
    inputBytes_  = newInputBytes;
    outputBytes_ = newOutputBytes;

    allocateOrReuseDevice_(bytesChanged);
}

/************************************************************************************************************************
 * @brief   显存分配/复用
 * @details 仅在 I/O 字节数变化时重新分配；否则复用旧缓冲
 * @param   bytesChanged   若为 true 则按当前字节数重新申请
 ***********************************************************************************************************************/
void TensorRTRunner::allocateOrReuseDevice_(bool bytesChanged)
{
    if (!allocated_ || bytesChanged)
    {
        if (dInput_)  { cudaFree(dInput_);  dInput_  = nullptr; }
        if (dOutput_) { cudaFree(dOutput_); dOutput_ = nullptr; }

        TRT_CHECK_CUDA(cudaMalloc(&dInput_,  inputBytes_));
        TRT_CHECK_CUDA(cudaMalloc(&dOutput_, outputBytes_));
        allocated_ = true;
    }
}

/************************************************************************************************************************
 * @brief   仅设置样本维（沿用当前 batch；若未初始化则默认 1）
 * @param   sampleDims  样本维（不含 batch）
 * @throw   std::runtime_error  绑定维度设置失败
 ***********************************************************************************************************************/
void TensorRTRunner::setInputShape(const std::vector<int>& sampleDims)
{
    const int batch = (currentBatch_ > 0) ? currentBatch_ : 1;
    applyInputDims_(sampleDims, batch);
}

/************************************************************************************************************************
 * @brief   设置输入样本维与批大小（显式 batch，batch 维在第 0 维）
 * @param   sampleDims  样本维（不含 batch），如 {220} 或 {3,224,224}
 * @param   batchSize   批大小（>0）
 * @throw   std::runtime_error  范围/绑定维度设置失败
 ***********************************************************************************************************************/
void TensorRTRunner::setInputShape(const std::vector<int>& sampleDims, int batchSize)
{
    applyInputDims_(sampleDims, batchSize);
}

/************************************************************************************************************************
 * @brief   仅调整批大小（保持样本维不变）
 * @param   batchSize   批大小（>0）
 * @throw   std::runtime_error  样本维未设置/超出 profile/设置失败
 ***********************************************************************************************************************/
void TensorRTRunner::setBatchSize(int batchSize)
{
    if (batchSize <= 0)
    {
        throw std::runtime_error("batchSize must be > 0");
    }

    auto in = context_->getBindingDimensions(inputIndex_);
    if (in.nbDims <= 0 || hasDynamicDim(in))
    {
        throw std::runtime_error("setBatchSize: sample dims not set; call setInputShape(...) first.");
    }

    std::vector<int> sample;
    sample.reserve(static_cast<size_t>(in.nbDims - 1));
    for (int i = 1; i < in.nbDims; ++i) sample.push_back(in.d[i]);

    applyInputDims_(sample, batchSize);
}

/************************************************************************************************************************
 * @brief   确保显存已按当前维度完成分配
 * @details 若维度未指定（动态模型未 setInputShape），则抛出错误
 * @throw   std::runtime_error  动态模型未设置输入维度
 ***********************************************************************************************************************/
void TensorRTRunner::ensureAllocated()
{
    if (allocated_)
    {
        return;
    }

    const auto inDims = context_->getBindingDimensions(inputIndex_);
    if (!isDimsSpecified(inDims))
    {
        throw std::runtime_error("Dynamic model requires setInputShape before infer.");
    }

    // 尺寸缓存未初始化但维度已指定（例如静态模型）：按当前绑定同步一次
    const size_t newInputCount  = volume(inDims);
    const auto   outDims        = context_->getBindingDimensions(outputIndex_);
    if (!isDimsSpecified(outDims))
    {
        throw std::runtime_error("ensureAllocated: output dims unspecified.");
    }
    const size_t newOutputCount = volume(outDims);

    inputCount_  = newInputCount;
    outputCount_ = newOutputCount;
    inputBytes_  = inputCount_  * sizeof(float);
    outputBytes_ = outputCount_ * sizeof(float);

    currentBatch_             = std::max(1, inDims.d[0]);
    inputElemsPerSample_      = inputCount_  / static_cast<size_t>(currentBatch_);
    outputElemsPerSample_     = outputCount_ / static_cast<size_t>(currentBatch_);

    allocateOrReuseDevice_(/*bytesChanged=*/true);
}

/************************************************************************************************************************
 * @brief   执行一次推理（H2D→enqueue→D2H）
 * @param   hostInput  主机侧连续输入
 * @param   out        主机侧输出向量（函数内 resize）
 ***********************************************************************************************************************/
void TensorRTRunner::runOnce_(const float* hostInput, std::vector<float>& out)
{
    const size_t expect = inputElemsPerSample_ * static_cast<size_t>(currentBatch_);
    if (expect != inputCount_)
    {
        throw std::runtime_error("runOnce_: internal dims mismatch");
    }

    /* H2D -----------------------------------------------------------------------------------------------------------*/
    TRT_CHECK_CUDA(cudaMemcpyAsync(dInput_, hostInput, inputBytes_, cudaMemcpyHostToDevice, stream_));

    /* 绑定并执行 -----------------------------------------------------------------------------------------------------*/
    void* bindings[2]{ dInput_, dOutput_ };

    if (!context_->enqueueV2(bindings, stream_, nullptr))
    {
        throw std::runtime_error("enqueueV2 failed");
    }

    /* D2H + 同步 ----------------------------------------------------------------------------------------------------*/
    out.resize(outputCount_);
    TRT_CHECK_CUDA(cudaMemcpyAsync(out.data(), dOutput_, outputBytes_, cudaMemcpyDeviceToHost, stream_));
    TRT_CHECK_CUDA(cudaStreamSynchronize(stream_));
}

/************************************************************************************************************************
 * @brief   批量推理（连续内存）
 * @param   batchInput    主机侧输入数据指针（float32，连续）
 * @param   batchSize     批大小（>0）
 * @param   batchOutput   主机侧输出向量（函数内调整大小并写入）
 * @throw   std::runtime_error  维度不匹配或执行失败
 ***********************************************************************************************************************/
void TensorRTRunner::infer(const float* batchInput, size_t batchSize, std::vector<float>& batchOutput)
{
    if (batchSize == 0)
    {
        throw std::runtime_error("infer: batchSize == 0");
    }

    ensureAllocated();

    if (static_cast<int>(batchSize) != currentBatch_)
    {
        setBatchSize(static_cast<int>(batchSize));
    }

    runOnce_(batchInput, batchOutput);
}

/************************************************************************************************************************
 * @brief   单样本便捷接口（batch=1）
 * @param   singleInput   单样本输入数据指针（float32）
 * @param   singleOutput  单样本输出向量（函数内调整大小并写入）
 * @throw   std::runtime_error  维度不匹配或执行失败
 ***********************************************************************************************************************/
void TensorRTRunner::infer(const float* singleInput, std::vector<float>& singleOutput)
{
    ensureAllocated();
    if (currentBatch_ != 1)
    {
        setBatchSize(1);
    }
    runOnce_(singleInput, singleOutput);
}

/************************************************************************************************************************
 * @brief   批量推理（非连续）：每样本一个指针，内部拼接为连续缓冲后一次性推理
 * @details
 *    1) 确保已分配
 *    2) 若 batch 不匹配则 setBatchSize()
 *    3) 将 sampleInputs[i] 依次拷贝到可复用 stagingHost_（按需扩容）
 *    4) 复用连续批量 runOnce_(stagingHost_.data(), ...)
 * @param   sampleInputs  大小为 batchSize 的样本指针数组；每个样本长度 = inputElementsPerSample()
 * @param   batchOutput   主机侧输出向量（函数内调整大小并写入）
 * @throw   std::runtime_error  维度未设置/空指针/拷贝或执行失败
 ***********************************************************************************************************************/
void TensorRTRunner::infer(const std::vector<const float*>& sampleInputs, std::vector<float>& batchOutput)
{
    const size_t bs = sampleInputs.size();
    if (bs == 0)
    {
        throw std::runtime_error("infer(sampleInputs): batchSize == 0");
    }

    ensureAllocated();

    if (inputElementsPerSample() == 0)
    {
        throw std::runtime_error("infer(sampleInputs): input shape not set; call setInputShape() first.");
    }

    if (static_cast<int>(bs) != currentBatch_)
    {
        setBatchSize(static_cast<int>(bs));
    }

    const size_t sampleElems = inputElementsPerSample();
    const size_t needElems   = sampleElems * bs;
    if (stagingHost_.size() < needElems)
    {
        stagingHost_.resize(needElems);
    }

    const size_t sampleBytes = sampleElems * sizeof(float);
    for (size_t i = 0; i < bs; ++i)
    {
        const float* src = sampleInputs[i];
        if (!src)
        {
            throw std::runtime_error("infer(sampleInputs): null sample pointer at index " + std::to_string(i));
        }
        std::memcpy(stagingHost_.data() + i * sampleElems, src, sampleBytes);
    }

    runOnce_(stagingHost_.data(), batchOutput);
}

/************************************************************************************************************************
 * @brief   获取当前输入/输出维度
 * @return  输入/输出维度（整型向量，含 batch）；若未指定则返回空
 ***********************************************************************************************************************/
std::vector<int> TensorRTRunner::getInputDims() const
{
    const auto d = context_->getBindingDimensions(inputIndex_);
    return dimsToVector(d);
}

std::vector<int> TensorRTRunner::getOutputDims() const
{
    const auto d = context_->getBindingDimensions(outputIndex_);
    return dimsToVector(d);
}

/************************************************************************************************************************
 * @brief   获取输入/输出绑定名称（调试用）
 ***********************************************************************************************************************/
std::string TensorRTRunner::getInputName()  const { return engine_->getBindingName(inputIndex_);  }
std::string TensorRTRunner::getOutputName() const { return engine_->getBindingName(outputIndex_); }

/************************************************************************************************************************
 * @brief   计算给定维度的元素总数
 * @param   d   TRT 维度
 * @return  元素数量（各维度连乘）
 ***********************************************************************************************************************/
size_t TensorRTRunner::volume(const nvinfer1::Dims& d)
{
    size_t v = 1;
    for (int i = 0; i < d.nbDims; ++i)
    {
        v *= static_cast<size_t>(d.d[i]);
    }
    return v;
}

/************************************************************************************************************************
 * @brief   将 TRT 维度转换为 std::vector<int>
 * @param   d   TRT 维度
 * @return  维度向量；若 nbDims<=0（未指定），返回空
 ***********************************************************************************************************************/
std::vector<int> TensorRTRunner::dimsToVector(const nvinfer1::Dims& d)
{
    if (d.nbDims <= 0)
    {
        return {};
    }
    std::vector<int> v(d.nbDims);
    for (int i = 0; i < d.nbDims; ++i)
    {
        v[i] = d.d[i];
    }
    return v;
}

/************************************************************************************************************************
 * @brief   动态/未指定维度判定
 * @param   d   TRT 维度
 * @return  存在 -1 返回 true；否则 false
 ***********************************************************************************************************************/
bool TensorRTRunner::hasDynamicDim(const nvinfer1::Dims& d) const
{
    for (int i = 0; i < d.nbDims; ++i)
    {
        if (d.d[i] < 0) return true;
    }
    return false;
}

/************************************************************************************************************************
 * @brief   维度是否“已指定”（nbDims>0 且无 -1）
 * @param   d   TRT 维度
 * @return  true/false
 ***********************************************************************************************************************/
bool TensorRTRunner::isDimsSpecified(const nvinfer1::Dims& d) const
{
    if (d.nbDims <= 0) return false;
    for (int i = 0; i < d.nbDims; ++i)
    {
        if (d.d[i] < 0) return false;
    }
    return true;
}

} // namespace trt
