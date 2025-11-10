/**
 * @file    TensorRTRunner.hpp
 * @brief   TensorRT 推理封装（单输入/单输出 + 精简批量接口，含非连续批量支持）
 *
 * @author  -
 * @date    2025-08-10
 * @version v1.6
 */

#pragma once
#include <NvInfer.h>
#include <cuda_runtime.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace trt
{

/************************************************************************************************************************
 * @brief   TensorRT 日志器
 * @details 仅输出 WARNING 及以上等级日志（彩色）；如需更详细日志，可放宽到 INFO/VERBOSE。
 ***********************************************************************************************************************/
class Logger final : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char* msg) noexcept override;
};

/* 工具宏/函数 ---------------------------------------------------------------------------------------------------------*/
/**
 * @brief   CUDA 错误检查
 * @details 发生错误时直接打印并中止进程；对实时系统可改为异常抛出/错误码返回。
 * @param   e       CUDA 返回码
 * @param   file    源文件名
 * @param   line    行号
 */
void checkCuda(cudaError_t e, const char* file, int line);

#define TRT_CHECK_CUDA(x) trt::checkCuda((x), __FILE__, __LINE__)

/************************************************************************************************************************
 * @brief   TensorRT 推理运行器（单输入/单输出，float32，支持动态形状与批量）
 * @details 职责：
 *          - 反序列化 engine（.plan）
 *          - 显式 batch 动态形状设置（batch 维在第 0 维）
 *          - 显存申请与复用
 *          - 推理执行（enqueueV2）
 * @note    统一推理接口 infer(...)：
 *          - infer(batchPtr, batchSize, out)     连续批量
 *          - infer(samplePtrsVec, out)           非连续批量（内部拼接为连续）
 *          - infer(singlePtr, out)               单样本便捷接口（batch=1）
 ************************************************************************************************************************/
class TensorRTRunner
{
public:
    /**
     * @brief   构造函数：加载并反序列化 engine
     * @param   enginePath  序列化的 .plan 文件路径
     * @throw   std::runtime_error  反序列化/资源创建失败
     */
    explicit TensorRTRunner(const std::string& enginePath);

    /**
     * @brief   析构：释放 CUDA 资源（流、显存）
     */
    ~TensorRTRunner();

    /* 形状设置 -------------------------------------------------------------------------------------------------------*/
    /**
     * @brief   设置输入样本维与批大小（显式 batch，batch 维在第 0 维）
     * @param   sampleDims  样本维（不含 batch），如 {220} 或 {3,224,224}
     * @param   batchSize   批大小（>0）
     * @throw   std::runtime_error  范围/绑定维度设置失败
     */
    void setInputShape(const std::vector<int>& sampleDims, int batchSize);

    /**
     * @brief   仅设置样本维（沿用当前 batch；若未初始化则默认 1）
     * @param   sampleDims  样本维（不含 batch）
     * @throw   std::runtime_error  绑定维度设置失败
     */
    void setInputShape(const std::vector<int>& sampleDims);

    /**
     * @brief   仅调整批大小（保持样本维不变）
     * @param   batchSize   批大小（>0）
     * @throw   std::runtime_error  样本维未设置/超出 profile/设置失败
     */
    void setBatchSize(int batchSize);

    /* 推理接口 -------------------------------------------------------------------------------------------------------*/
    /**
     * @brief   批量推理（连续内存）
     * @param   batchInput   指向连续输入（长度 = batchSize * inputElementsPerSample()）
     * @param   batchSize    批大小（>0）
     * @param   batchOutput  扁平化输出（函数内 resize）
     * @throw   std::runtime_error  维度未设置/拷贝或执行失败
     */
    void infer(const float* batchInput, size_t batchSize, std::vector<float>& batchOutput);

    /**
     * @brief   单样本便捷接口（内部以 batch=1 执行）
     * @param   singleInput  单样本输入
     * @param   singleOutput 单样本输出
     * @throw   std::runtime_error  维度未设置/拷贝或执行失败
     */
    void infer(const float* singleInput, std::vector<float>& singleOutput);

    /**
     * @brief   批量推理（非连续）：每样本一个指针，内部拼接为连续缓冲后一次性推理
     * @param   sampleInputs  大小为 batchSize 的样本指针数组
     * @param   batchOutput   扁平化输出（函数内 resize）
     * @throw   std::runtime_error  维度未设置/空指针/拷贝或执行失败
     */
    void infer(const std::vector<const float*>& sampleInputs, std::vector<float>& batchOutput);

    /* 查询接口 -------------------------------------------------------------------------------------------------------*/
    /**
     * @brief   获取当前绑定输入/输出维度（含 batch）；若未指定则返回空
     */
    std::vector<int> getInputDims()  const;
    std::vector<int> getOutputDims() const;

    /**
     * @brief   获取绑定名称（调试用）
     */
    std::string getInputName()  const;
    std::string getOutputName() const;

    /**
     * @brief   每样本元素数（不含 batch）/ 当前批大小
     */
    size_t inputElementsPerSample()  const { return inputElemsPerSample_;  }
    size_t outputElementsPerSample() const { return outputElemsPerSample_; }
    int    currentBatch()            const { return currentBatch_; }

private:
    /* 工具：维度与体积 -----------------------------------------------------------------------------------------------*/
    static size_t            volume(const nvinfer1::Dims& d);
    static std::vector<int>  dimsToVector(const nvinfer1::Dims& d);
    static std::vector<char> readFile(const std::string& path);

    /* 内部流程：维度/显存管理 -----------------------------------------------------------------------------------------*/
    void validateBindingsAndTypes();                     // 一次性校验：1 入 1 出 + FP32
    void validateWithinProfile(const nvinfer1::Dims& d); // 校验维度是否在当前 profile 范围（动态）
    bool hasDynamicDim(const nvinfer1::Dims& d) const;   // 是否存在 -1（动态）
    bool isDimsSpecified(const nvinfer1::Dims& d) const; // nbDims>0 且不含 -1

    /**
     * @brief   应用输入维度（统一入口）
     * @details 维度构造 → Profile 校验 → setBinding → 校验 in/out → 若尺寸变化才重分配
     * @param   sampleDims  样本维（不含 batch）
     * @param   batchSize   批大小（>0）
     * @throw   std::runtime_error  越界/绑定失败/维度未指定
     */
    void applyInputDims_(const std::vector<int>& sampleDims, int batchSize);

    /**
     * @brief   显存分配/复用
     * @details 仅在 I/O 字节数变化时重新分配；否则复用旧缓冲
     * @param   bytesChanged   若为 true 则按当前字节数重新申请
     */
    void allocateOrReuseDevice_(bool bytesChanged);

    /**
     * @brief   确保已按维度完成显存分配
     * @throw   std::runtime_error  动态模型未设置输入维度
     */
    void ensureAllocated();

    /**
     * @brief   执行一次推理（H2D→enqueue→D2H）
     * @param   hostInput  主机侧连续输入
     * @param   out        主机侧输出向量（函数内 resize）
     */
    void runOnce_(const float* hostInput, std::vector<float>& out);

private:
    /* TensorRT 资源 ---------------------------------------------------------------------------------------------------*/
    Logger                                       gLogger_;     // 日志器
    std::unique_ptr<nvinfer1::IRuntime>          runtime_;     // 反序列化运行时
    std::unique_ptr<nvinfer1::ICudaEngine>       engine_;      // 引擎
    std::unique_ptr<nvinfer1::IExecutionContext> context_;     // 执行上下文

    int inputIndex_  = -1;                                   // 输入绑定索引
    int outputIndex_ = -1;                                   // 输出绑定索引

    /* CUDA 资源 -------------------------------------------------------------------------------------------------------*/
    cudaStream_t stream_ = nullptr;                          // 推理流
    void* dInput_  = nullptr;                                // 设备侧输入缓冲
    void* dOutput_ = nullptr;                                // 设备侧输出缓冲
    bool  allocated_ = false;                                // 显存是否已分配

    /* I/O 尺寸信息 ---------------------------------------------------------------------------------------------------*/
    size_t inputCount_  = 0;                                 // 输入元素总数（含 batch）
    size_t outputCount_ = 0;                                 // 输出元素总数（含 batch）
    size_t inputBytes_  = 0;                                 // 输入字节数
    size_t outputBytes_ = 0;                                 // 输出字节数

    size_t inputElemsPerSample_  = 0;                        // 每样本输入元素数（不含 batch）
    size_t outputElemsPerSample_ = 0;                        // 每样本输出元素数（不含 batch）
    int    currentBatch_         = 1;                        // 当前 batch

    /* Host 侧可复用 staging 缓冲（非连续批量时使用） -----------------------------------------------------------------*/
    std::vector<float> stagingHost_;                         // 容量不足时扩容（只增不减）
};

} // namespace trt
