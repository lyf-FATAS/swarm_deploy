/**
 * @file    inferance_benchmark.cpp
 * @brief   TensorRT 批量连续推理基准（支持 batch=1 及批量；持续循环，实时打印速度，Ctrl+C 退出）
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "TensorRTRunner.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using SteadyClock = std::chrono::steady_clock;
using ms_f  = std::chrono::duration<double, std::milli>;

static std::atomic<bool> g_stop{false};

/************************************************************************************************************************
 * @brief   信号处理：Ctrl+C 设置停止标志
 ***********************************************************************************************************************/
void handle_sigint(int)
{
    g_stop.store(true);
}

/************************************************************************************************************************
 * @brief   入口：持续执行推理并统计速度（支持批量，包含 batch=1）
 ***********************************************************************************************************************/
int main()
{
    /* 安装 Ctrl+C 处理 -----------------------------------------------------------------------------------------------*/
    std::signal(SIGINT, handle_sigint);

    /* 配置你的输入形状与模型路径 -----------------------------------------------------------------------------------*/
    std::vector<int> input_shape = {220};                     // 样本维（不含 batch）
    std::string model_path = "/home/nv/SWARM-Physical/src/drone_control/model/acc_8_3_rand_b1_32.engine";  // .engine 路径
    // std::string model_path = "/home/nv/SWARM-Physical/src/drone_control/model/acc_8_3_rand.engine";  // .engine 路径

    /* 配置要测试的批大小（若引擎固定为 1，会自动回退并提示） ------------------------------------------------------*/
    int desired_batch = 32;  // ← 想测多少填多少：1/2/4/8/16...

    try
    {
        /* 1. 创建 runner，加载 plan --------------------------------------------------------------------------------*/
        trt::TensorRTRunner runner(model_path);

        /* 2. 查询绑定名与初始维度（用于信息输出） ------------------------------------------------------------------*/
        std::cout << "Input name : " << runner.getInputName()  << "\n";
        std::cout << "Output name: " << runner.getOutputName() << "\n";

        auto inDims  = runner.getInputDims();   // 可能含 -1
        /* 3. 设置实际输入形状与批大小（关键：必须先把动态维度定下来） ----------------------------------------------*/
        const bool input_is_dynamic = std::any_of(inDims.begin(), inDims.end(), [](int x){ return x < 0; });
        if (input_is_dynamic)
        {
            // 动态模型：先定样本维 + 批大小
            runner.setInputShape(input_shape, desired_batch);
            inDims  = runner.getInputDims();  // 现在无 -1
        }
        else
        {
            // 静态模型：引擎已固定 batch（常见是 1）
            const int engine_batch = (inDims.size() > 0 ? inDims[0] : 1);
            if (engine_batch != desired_batch)
            {
                if (engine_batch == 1 && desired_batch > 1)
                {
                    // 上色 WARN：黄色 [WARN]，加粗 fallback，青色建议参数
                    std::cout
                        << "\033[33m[WARN]"
                        << "Engine profile fixes batch=1; "
                        << "fallback to batch=1. "
                        << "Rebuild engine with --min/opt/max shapes to enable batching.\n\033[0m";
                }
                desired_batch = engine_batch;  // 与引擎保持一致（否则 TRT 会报错）
            }
            // 静态模型不需要 setInputShape
        }
        auto outDims = runner.getOutputDims();
        // 打印输入形状
        std::cout << "Input dims : [";
        for (size_t i = 0; i < inDims.size(); ++i)
        {
            std::cout << inDims[i];
            if (i + 1 != inDims.size()) std::cout << ", ";
        }
        std::cout << "]\n";
        // 打印输出形状
        std::cout << "Output dims: [";
        for (size_t i = 0; i < outDims.size(); ++i)
        {
            std::cout << outDims[i];
            if (i + 1 != outDims.size()) std::cout << ", ";
        }
        std::cout << "]\n";

        /* 4. 预分配输入/输出缓冲（必须在形状已确定后再计算 inputCount） --------------------------------------------*/
        const size_t inputCount =
            std::accumulate(inDims.begin(), inDims.end(), size_t{1}, std::multiplies<size_t>());

        std::vector<float> input(inputCount);
        // 示例数据：只初始化一次，后续重复使用，避免影响计时
        std::iota(input.begin(), input.end(), 0.0f);

        std::vector<float> output;  // runner.infer 内部会 resize

        /* 5. 预热（可调），避免首轮缓存/加载影响统计 ---------------------------------------------------------------*/
        const int warmup_iters = 50;
        for (int i = 0; i < warmup_iters; ++i)
        {
            // 注意：这里的第二个参数是“批大小”，不是元素个数
            runner.infer(input.data(), /*batchSize=*/static_cast<size_t>(desired_batch), output);
        }

        /* 6. 基准循环：持续到 Ctrl+C，周期性打印 -------------------------------------------------------------------*/
        std::cout << "=== Benchmark running... (Ctrl+C to stop)"
                  << " | batch=" << desired_batch
                  << " | in_elems_per_sample=" << runner.inputElementsPerSample()
                  << " | out_elems_per_sample=" << runner.outputElementsPerSample()
                  << " ===\n";

        auto t_begin       = SteadyClock::now();
        auto t_last_report = t_begin;

        // 总体统计
        uint64_t total_iters   = 0;
        double   total_ms      = 0.0;

        // 区间统计（每秒打印一次）
        uint64_t window_iters  = 0;
        double   window_ms     = 0.0;
        const std::chrono::seconds report_interval(1);

        while (!g_stop.load())
        {
            auto t0 = SteadyClock::now();

            // 连续批量推理（H2D -> enqueueV2 -> D2H）
            runner.infer(input.data(), /*batchSize=*/static_cast<size_t>(desired_batch), output);

            auto t1 = SteadyClock::now();

            const double elapsed_ms = std::chrono::duration_cast<ms_f>(t1 - t0).count();

            // 累加统计
            ++total_iters;
            total_ms  += elapsed_ms;

            ++window_iters;
            window_ms += elapsed_ms;

            // 周期性打印（1 秒）
            if (t1 - t_last_report >= report_interval)
            {
                const double avg_ms_window = (window_iters > 0) ? (window_ms / window_iters) : 0.0;
                const double ips_window    = (avg_ms_window > 0.0) ? (1000.0 / avg_ms_window) : 0.0;

                std::cout
                    << "[STAT] window: iters=" << window_iters
                    << ", avg=" << avg_ms_window << " ms"
                    << ", FPS=" << ips_window
                    << "\n";

                // 清零区间统计
                t_last_report = t1;
                window_iters  = 0;
                window_ms     = 0.0;
            }
        }

        /* 7. 收尾与总统计 -------------------------------------------------------------------------------------------*/
        auto t_end = SteadyClock::now();
        const double total_s = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin).count();
        const double avg_ms  = (total_iters > 0) ? (total_ms / total_iters) : 0.0;
        const double fps     = (avg_ms > 0.0) ? (1000.0 / avg_ms) : 0.0;

        std::cout << "=== Benchmark stopped ===\n";
        std::cout << "Total time: " << total_s << " s, "
                  << "iters: " << total_iters << ", "
                  << "avg: " << avg_ms << " ms, "
                  << "FPS: " << fps << "\n";

        // 可选：打印少量输出检查正确性
        if (!output.empty())
        {
            std::cout << "Sample outputs: ";
            for (size_t i = 0; i < std::min<size_t>(output.size(), 8); ++i)
            {
                std::cout << output[i] << (i + 1 < std::min<size_t>(output.size(), 8) ? ", " : "\n");
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
