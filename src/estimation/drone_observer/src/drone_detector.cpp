/**
 * @file    drone_detector.cpp
 * @brief   基于 Ultralytics YOLO 的多路相机批量检测器（ROS1，4 路同步）
 *
 * @author  -
 * @date    2025-08-19
 * @version v1.6
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "drone_detector.hpp"

/* 标准库 --------------------------------------------------------------------------------------------------------------*/
#include <algorithm>
#include <array>
#include <numeric>
#include <tuple>
#include <stdexcept>
#include <string>
#include <vector>

#include <memory>

/* 命名空间 ------------------------------------------------------------------------------------------------------------*/
namespace observer
{

/************************************************************************************************************************
 * @brief   构造函数：初始化 YOLO 批量检测器
 * @details 加载 ROS 参数、模型、订阅/发布器与同步器，必要时创建窗口。
 ***********************************************************************************************************************/
YoloBatchDetector::YoloBatchDetector(ros::NodeHandle& nh)
: nh_(nh), it_(nh), last_t0_(ros::Time::now())
{
    // 读取 ROS 参数
    nh.param("print_debug", print_debug_, true);
    nh.param("conf_threshold", conf_threshold_, 0.5);
    nh.param<std::string>("model_path", model_path_, "/home/nv/SWARM-Physical/src/drone_observer/model/yolo11_8_17_b1_4_fp32.engine");

    // 默认话题配置
    std::vector<std::string> default_image_topics{
        "/front/right/image_raw", "/right/right/image_raw",
        "/back/right/image_raw" , "/left/right/image_raw"
    };
    std::vector<std::string> default_detected_topics{
        "/yolo/front/right/detected_image", "/yolo/right/right/detected_image",
        "/yolo/back/right/detected_image" , "/yolo/left/right/detected_image"
    };

    nh.param("image_topics", image_topics_, default_image_topics);
    nh.param("detected_topics", detected_topics_, default_detected_topics);

    if (image_topics_.size() != 4 || detected_topics_.size() != 4)
    {
        throw std::runtime_error("话题数量必须为 4");
    }

    std::vector<int> input_shape = {3, 640, 640};
    initInference_(model_path_, input_shape, batch_num_);   // 加载模型
    initSubscribers_();    // 初始化订阅器
    initPublishers_();     // 初始化发布器
    initSynchronizer_();   // 初始化同步策略（4 路图像）

    ROS_INFO_STREAM("\033[32m[drone_detector] Init completely!\033[0m");
}

/************************************************************************************************************************
 * @brief   析构函数：清理资源
 ***********************************************************************************************************************/
YoloBatchDetector::~YoloBatchDetector() = default;

/************************************************************************************************************************
 * @brief   初始化 TensorRT 推理
 ***********************************************************************************************************************/
void YoloBatchDetector::initInference_(std::string& model_path, std::vector<int>& input_shape, int desired_batch)
{
    /* 1. 创建 model，加载 plan -------------------------------------------------------------------------*/
    model_ = std::make_unique<trt::TensorRTRunner>(model_path);

    /* 2. 查询绑定名与初始维度（用于信息输出） --------------------------------------------------------------*/
    ROS_INFO_STREAM("[drone_detector] [TensorRT] Input name : " << model_->getInputName());
    ROS_INFO_STREAM("[drone_detector] [TensorRT] Output name: " << model_->getOutputName());

    /* 3. 设置实际输入形状与批大小（关键：必须先把动态维度定下来） ----------------------------------------------*/
    auto in_dims  = model_->getInputDims();   // 可能含 -1
    const bool input_is_dynamic = std::any_of(in_dims.begin(), in_dims.end(), [](int x){ return x < 0; });
    if (input_is_dynamic)
    {
        // 动态模型：先定样本维 + 批大小
        model_->setInputShape(input_shape, desired_batch);
        in_dims  = model_->getInputDims();  // 现在无 -1
    }
    else
    {
        // 静态模型：引擎已固定 batch（常见是 1）
        const int engine_batch = (in_dims.size() > 0 ? in_dims[0] : 1);
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
    auto out_dims = model_->getOutputDims();

    /* 4. 相关信息打印 -----------------------------------------------------------------------------------*/
    std::ostringstream oss;
    oss << "[TensorRT] Input dims : [";
    for (size_t i = 0; i < in_dims.size(); ++i)
    {
        oss << in_dims[i] << (i + 1 != in_dims.size() ? ", " : "");
    }
    oss << "]";
    ROS_INFO_STREAM("[drone_detector] " << oss.str());

    std::ostringstream oss2;
    oss2 << "[TensorRT] Output dims: [";
    for (size_t i = 0; i < out_dims.size(); ++i)
    {
        oss2 << out_dims[i] << (i + 1 != out_dims.size() ? ", " : "");
    }
    oss2 << "]";
    ROS_INFO_STREAM("[drone_detector] " << oss2.str());
}

// 初始化 4 路图像订阅器（message_filters::Subscriber 版本）
void YoloBatchDetector::initSubscribers_()
{
    subs_.clear();
    subs_.reserve(4);
    for (size_t i = 0; i < 4; ++i)
    {
        subs_.emplace_back(
            std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh_, image_topics_[i], 1)
        );
        ROS_INFO_STREAM("[drone_detector] [Sub] : " << image_topics_[i]);
    }
}

// 初始化 4 路图像发布器（用于发布检测结果）
void YoloBatchDetector::initPublishers_()
{
    pubs_.resize(4);
    for (size_t i = 0; i < 4; ++i)
    {
        pubs_[i] = it_.advertise(detected_topics_[i], 1);
        ROS_INFO_STREAM("[drone_detector] [Pub] : " << detected_topics_[i]);
    }
}

// 初始化 4 路图像同步器（ApproximateTime）
void YoloBatchDetector::initSynchronizer_()
{
    try
    {
        int    queue_size = 1;      // 同步器队列
        double slop_sec   = 0.05;   // 匹配时间窗（50ms）

        sync_ = std::make_shared<Sync>(ApproxPolicy(queue_size), *subs_[0], *subs_[1], *subs_[2], *subs_[3]);
        sync_->setMaxIntervalDuration(ros::Duration(slop_sec));
        sync_->registerCallback(boost::bind(&YoloBatchDetector::callback, this, _1, _2, _3, _4));
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("message sync init failed : %s", e.what());
        return;
    }
}

/************************************************************************************************************************
 * @brief   Letterbox 预处理（将任意尺寸图像等比缩放到 dst 内，并在四周填充颜色）
 * @details
 *  - 按比例缩放，保持长宽比不变；不足部分用 `letterbox_color_` 填充。
 *  - 插值策略：缩小时用 INTER_AREA，放大时用 INTER_LINEAR（更优的视觉与速度折中）。
 *  - 填充对齐：把向下取整后的左/上边距作为返回 pad（dw, dh），右/下由剩余像素补齐，避免 1px 漏洞。
 *  - 返回：
 *      1) output  : 尺寸 = dst（如 640×640），类型与输入一致
 *      2) scale   : 缩放比例（min(dst.w / w, dst.h / h)）
 *      3) pad     : 左上角填充偏移（dw, dh），用于把模型输出还原回原图坐标
 * @param   img     输入图像（任意尺寸，常用 BGR8）
 * @param   dst     目标尺寸（如 {letterbox_w_, letterbox_h_}）
 * @return  std::tuple<cv::Mat, float, cv::Point> 见上
 * @throw   std::invalid_argument  输入为空或 dst 非法
 ***********************************************************************************************************************/
std::tuple<cv::Mat, float, cv::Point> YoloBatchDetector::letterbox_(const cv::Mat& img, const cv::Size& dst)
{
    if (img.empty())
        throw std::invalid_argument("letterbox_: input image is empty");
    if (dst.width <= 0 || dst.height <= 0)
        throw std::invalid_argument("letterbox_: invalid dst size");

    const int w = img.cols;
    const int h = img.rows;

    // 计算缩放比例
    const float sx = static_cast<float>(dst.width)  / static_cast<float>(w);
    const float sy = static_cast<float>(dst.height) / static_cast<float>(h);
    const float scale = std::min(sx, sy);

    // 目标内的等比尺寸
    int nw = std::max(1, static_cast<int>(std::round(w * scale)));
    int nh = std::max(1, static_cast<int>(std::round(h * scale)));

    // 选择插值方式：缩小时 AREA，放大时 LINEAR
    const int interp = (scale < 1.0f) ? cv::INTER_AREA : cv::INTER_LINEAR;

    // 先缩放到 (nw, nh)
    cv::Mat resized;
    if (nw == w && nh == h)
    {
        resized = img; // 零拷贝视图
    }
    else
    {
        cv::resize(img, resized, cv::Size(nw, nh), 0.0, 0.0, interp);
    }

    // 计算左右/上下填充（左上尽量均分，右下补齐余数，避免 1px 漏洞）
    const int dw_left   = (dst.width  - nw) / 2;
    const int dh_top    = (dst.height - nh) / 2;
    const int dw_right  = (dst.width  - nw) - dw_left;
    const int dh_bottom = (dst.height - nh) - dh_top;

    // 生成输出并拷贝
    cv::Mat output;
    if (dw_left == 0 && dh_top == 0 && dw_right == 0 && dh_bottom == 0)
    {
        // 恰好无填充，直接返回缩放图（或原图）
        output = (resized.data ? resized : img);
    }
    else
    {
        // 用固定颜色填充到目标尺寸
        cv::copyMakeBorder(
            resized.data ? resized : img, output,
            dh_top, dh_bottom, dw_left, dw_right,
            cv::BORDER_CONSTANT, letterbox_color_
        );
    }

    // 保险：确保尺寸就是 dst
    if (output.cols != dst.width || output.rows != dst.height)
        cv::resize(output, output, dst, 0.0, 0.0, interp);

    // 返回左上 pad（用于坐标反变换）
    return { output, scale, cv::Point(dw_left, dh_top) };
}

/************************************************************************************************************************
 * @brief   Mat(BGR8,HxW) → NCHW(float)（单线程高效版，无 split/merge）
 * @details
 *  - 一次遍历完成 BGR→(可选RGB)→float(可选0~1)→NCHW
 *  - 连续内存走紧凑分支：指针递增 + 4 像素轻度展开，减少索引开销
 *  - 非连续走逐行分支，仍使用指针前移
 ***********************************************************************************************************************/
inline void YoloBatchDetector::toTensorNCHW_(const cv::Mat& imgBGR8, bool rgb, bool norm01,
                                             float* __restrict dst, int C, int H, int W)
{
    assert(imgBGR8.type() == CV_8UC3);
    const int HW = H * W;
    float* __restrict d0 = dst + 0 * HW;
    float* __restrict d1 = dst + 1 * HW;
    float* __restrict d2 = dst + 2 * HW;

    const float scale = norm01 ? (1.f / 255.f) : 1.f;

    if (imgBGR8.isContinuous())
    {
        const uint8_t* __restrict p = imgBGR8.ptr<uint8_t>(0);
        // 指针就地推进，少算乘法和索引
        float* __restrict q0 = d0;
        float* __restrict q1 = d1;
        float* __restrict q2 = d2;

        int i = 0;
        const int N = HW;

        // 4 像素轻度展开
        for (; i <= N - 4; i += 4)
        {
            // #0
            uint8_t b0 = *p++; uint8_t g0 = *p++; uint8_t r0 = *p++;
            *q0++ = (rgb ? r0 : b0) * scale;
            *q1++ = g0 * scale;
            *q2++ = (rgb ? b0 : r0) * scale;
            // #1
            uint8_t b1 = *p++; uint8_t g1 = *p++; uint8_t r1 = *p++;
            *q0++ = (rgb ? r1 : b1) * scale;
            *q1++ = g1 * scale;
            *q2++ = (rgb ? b1 : r1) * scale;
            // #2
            uint8_t b2 = *p++; uint8_t g2 = *p++; uint8_t r2 = *p++;
            *q0++ = (rgb ? r2 : b2) * scale;
            *q1++ = g2 * scale;
            *q2++ = (rgb ? b2 : r2) * scale;
            // #3
            uint8_t b3 = *p++; uint8_t g3 = *p++; uint8_t r3 = *p++;
            *q0++ = (rgb ? r3 : b3) * scale;
            *q1++ = g3 * scale;
            *q2++ = (rgb ? b3 : r3) * scale;
        }
        for (; i < N; ++i)
        {
            uint8_t b = *p++; uint8_t g = *p++; uint8_t r = *p++;
            *q0++ = (rgb ? r : b) * scale;
            *q1++ = g * scale;
            *q2++ = (rgb ? b : r) * scale;
        }
    }
    else
    {
        for (int y = 0; y < H; ++y)
        {
            const uint8_t* __restrict row = imgBGR8.ptr<uint8_t>(y);
            float* __restrict q0 = d0 + y * W;
            float* __restrict q1 = d1 + y * W;
            float* __restrict q2 = d2 + y * W;

            int x = 0;
            // 4 像素轻度展开
            for (; x <= W - 4; x += 4)
            {
                // #0
                uint8_t b0 = row[3*x+0], g0 = row[3*x+1], r0 = row[3*x+2];
                q0[0] = (rgb ? r0 : b0) * scale;
                q1[0] = g0 * scale;
                q2[0] = (rgb ? b0 : r0) * scale;
                // #1
                uint8_t b1 = row[3*(x+1)+0], g1 = row[3*(x+1)+1], r1 = row[3*(x+1)+2];
                q0[1] = (rgb ? r1 : b1) * scale;
                q1[1] = g1 * scale;
                q2[1] = (rgb ? b1 : r1) * scale;
                // #2
                uint8_t b2 = row[3*(x+2)+0], g2 = row[3*(x+2)+1], r2 = row[3*(x+2)+2];
                q0[2] = (rgb ? r2 : b2) * scale;
                q1[2] = g2 * scale;
                q2[2] = (rgb ? b2 : r2) * scale;
                // #3
                uint8_t b3 = row[3*(x+3)+0], g3 = row[3*(x+3)+1], r3 = row[3*(x+3)+2];
                q0[3] = (rgb ? r3 : b3) * scale;
                q1[3] = g3 * scale;
                q2[3] = (rgb ? b3 : r3) * scale;

                q0 += 4; q1 += 4; q2 += 4;
            }
            for (; x < W; ++x)
            {
                uint8_t b = row[3*x+0], g = row[3*x+1], r = row[3*x+2];
                *q0++ = (rgb ? r : b) * scale;
                *q1++ = g * scale;
                *q2++ = (rgb ? b : r) * scale;
            }
        }
    }
}

/************************************************************************************************************************
 * @brief   回调函数：4 路图像处理 + 批量推理 + 后处理发布
 * @details
 *    1) 将 ROS 图像消息转换为 OpenCV 图像（BGR8）
 *    2) 对图像进行 letterbox 预处理（缩放 + 填充）
 *    3) 执行一次性批量推理（model_->predict）
 *    4) 结果框坐标映射回原图并绘制、标注
 *    5) 发布结果并（可选）显示图像
 ***********************************************************************************************************************/
void YoloBatchDetector::callback(const sensor_msgs::ImageConstPtr& i1,
                                 const sensor_msgs::ImageConstPtr& i2,
                                 const sensor_msgs::ImageConstPtr& i3,
                                 const sensor_msgs::ImageConstPtr& i4)
{
    auto t0 = ros::Time::now();

    /* 基本配置 -------------------------------------------------------------------------------------------------------*/
    constexpr size_t batch_num = 4;
    const std::array<const sensor_msgs::ImageConstPtr, batch_num> msgs{ i1, i2, i3, i4 };

    /* 预分配容器：避免反复扩容 ---------------------------------------------------------------------------------------*/
    raws_.clear();          raws_.reserve(batch_num);
    perprocessed_.clear();  perprocessed_.reserve(batch_num);
    scales_.clear();        scales_.reserve(batch_num);
    pads_.clear();          pads_.reserve(batch_num);
    // headers_.clear();       headers_.reserve(batch_num);

    /* 获取模型输入尺寸信息 -------------------------------------------------------------------------------------------*/
    const auto in_dims = model_->getInputDims();
    const int  C = (in_dims.size() >= 3) ? in_dims[in_dims.size() - 3] : 3;
    const int  H = (in_dims.size() >= 2) ? in_dims[in_dims.size() - 2] : 640;
    const int  W = (in_dims.size() >= 1) ? in_dims[in_dims.size() - 1] : 640;
    const size_t elems_per_sample = model_->inputElementsPerSample();   // = C*H*W
    const size_t total_elems = batch_num * elems_per_sample;

    // 预分配/复用：避免反复扩容
    if (input_.size() != total_elems) input_.resize(total_elems);

    /* 1. ROS 图像 → OpenCV + 预处理（letterbox）*/
    for (const auto& m : msgs)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(m, sensor_msgs::image_encodings::BGR8);
            // headers_.emplace_back(cv_ptr->header);
            raws_.emplace_back(cv_ptr->image);  // 不 clone，零拷贝视图；如需隔离可改为 .clone()

            auto [resized, scale, pad] = letterbox_(raws_.back(), cv::Size(W, H)); // 你已有的接口（返回: Mat/缩放/填充）
            perprocessed_.emplace_back(std::move(resized));
            scales_.emplace_back(scale);
            pads_.emplace_back(pad);
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge 异常：%s", e.what());
            return;
        }
    }

    /* 2.逐样本写入大数组（按 NCHW 连续布局） */
    for (size_t k = 0; k < batch_num; ++k)
    {
        float* dst = input_.data() + k * elems_per_sample;
        // BGR8 → RGB Float32 (0~1)，并排列为 NCHW
        toTensorNCHW_(perprocessed_[k], /*RGB=*/true, /*norm01=*/true, dst, C, H, W);
    }

    /* 3. 批量推理 */
    auto t1 = ros::Time::now();
    model_->infer(input_.data(), /*batchSize=*/static_cast<size_t>(batch_num), output_);
    auto t2 = ros::Time::now();

    // // 3. 后处理 + 可视化 + 发布
    // const size_t N = std::min(results.size(), raws.size());
    // for (size_t i = 0; i < N; ++i)
    // {
    //     const auto& dets = results[i];
    //     cv::Mat drawn = raws[i].clone();
    //     const auto& scale = scales[i];
    //     const auto& pad = pads[i];
    //     const int cols = drawn.cols, rows = drawn.rows;

    //     for (const auto& det : dets.boxes)
    //     {
    //         int x1 = std::max(0, static_cast<int>((det.x1 - pad.x) / scale));
    //         int y1 = std::max(0, static_cast<int>((det.y1 - pad.y) / scale));
    //         int x2 = std::min(cols - 1, static_cast<int>((det.x2 - pad.x) / scale));
    //         int y2 = std::min(rows - 1, static_cast<int>((det.y2 - pad.y) / scale));

    //         cv::rectangle(drawn, {x1, y1}, {x2, y2}, {0, 255, 0}, 2);
    //         std::string label = model_->names[det.class_id] + ":" + cv::format("%.2f", det.conf);
    //         cv::putText(drawn, label, {x1, std::max(0, y1 - 5)}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1);
    //     }

    //     try
    //     {
    //         auto msg = cv_bridge::CvImage(headers[i], "bgr8", drawn).toImageMsg();
    //         pubs_[i].publish(msg);
    //         if (show_window_) cv::imshow("YOLO 检测结果 " + std::to_string(i), drawn);
    //     }
    //     catch (const std::exception& e)
    //     {
    //         ROS_ERROR("图像发布失败：%s", e.what());
    //     }
    // }

    /* 4. 调试与时间统计 */
    auto t3 = ros::Time::now();
    const double infer_time    = (t2 - t1).toSec() * 1000.0;
    const double callback_time = (t3 - t0).toSec() * 1000.0;
    const double callback_freq = 1.0 / (t0 - last_t0_).toSec();
    last_t0_ = t0;

    if (print_debug_)
    {
        std::cout << "================ Time Consume ===============\n"
                  << std::fixed << std::setprecision(3)
                  << "Infer time   : " << infer_time    << " ms\n"
                  << "Callback time: " << callback_time << " ms\n"
                  << "Callback freq: " << callback_freq << " Hz\n"
                  << "==============================================\n\n\n";
    }
}

} // namespace observer

/************************************************************************************************************************
 * @brief   主函数：ROS 节点入口
 * @details 初始化节点、实例化检测器并阻塞循环（ros::spin）。建议使用私有命名空间 NodeHandle("~")
 ***********************************************************************************************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_detector");
    ros::NodeHandle nh("~");

    try
    {
        observer::YoloBatchDetector detector(nh);
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("检测器异常：%s", e.what());
        return 1;
    }

    return 0;
}
