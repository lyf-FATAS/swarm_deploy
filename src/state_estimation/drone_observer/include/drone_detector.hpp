/**
 * @file    drone_detector.hpp
 * @brief   基于 YOLO 的 ROS 多路相机批量检测器（4 路同步）
 *
 * @author  -
 * @date    2025-08-19
 * @version v1.6
 */

#pragma once

/* ROS 相关 -----------------------------------------------------------------------------------------------------------*/
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/* OpenCV -------------------------------------------------------------------------------------------------------------*/
#include <opencv2/opencv.hpp>

/* TensorRT 推理 ------------------------------------------------------------------------------------------------------*/
#include "TensorRTRunner.hpp"

/* 标准库 -------------------------------------------------------------------------------------------------------------*/
#include <memory>
#include <string>
#include <tuple>
#include <vector>

using ApproxPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>;
using Sync = message_filters::Synchronizer<ApproxPolicy>;

namespace observer
{

/************************************************************************************************************************
 * @brief   YOLO 批量检测器（支持 4 路图像同步）
 ***********************************************************************************************************************/
class YoloBatchDetector
{
public:
    /**
     * @brief 构造函数：初始化检测器
     * @param nh ROS 节点句柄（建议为私有命名空间）
     * @throw std::runtime_error 初始化失败
     */
    explicit YoloBatchDetector(ros::NodeHandle& nh);

    /**
     * @brief 析构函数：关闭窗口、清理资源
     */
    ~YoloBatchDetector();

    /**
     * @brief 回调函数：处理 4 路相机输入并执行批量检测
     */
    void callback(const sensor_msgs::ImageConstPtr& i1,
                  const sensor_msgs::ImageConstPtr& i2,
                  const sensor_msgs::ImageConstPtr& i3,
                  const sensor_msgs::ImageConstPtr& i4);

private:
    /**
     * @brief 图像预处理：letterbox（保持比例缩放 + 居中填充）
     * @param img 原始图像
     * @return (预处理图像, 缩放比例, 填充偏移)
     */
    std::tuple<cv::Mat, float, cv::Point> letterbox_(const cv::Mat& img, const cv::Size& dst);

    inline void toTensorNCHW_(const cv::Mat& imgBGR8, bool rgb, bool norm01,
                              float* __restrict dst, int C, int H, int W);

    /**
     * @brief 初始化 TensorRT 推理
     */
    void initInference_(std::string& model_path, std::vector<int>& input_shape, int desired_batch);

    /**
     * @brief 初始化图像订阅者
     */
    void initSubscribers_();

    /**
     * @brief 初始化图像发布者
     */
    void initPublishers_();

    /**
     * @brief 初始化同步器（4 路近似时间）
     */
    void initSynchronizer_();

private:
    /* ROS 资源 -------------------------------------------------------------------------------------------------------*/
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    std::vector<std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>> subs_;
    std::shared_ptr<message_filters::Synchronizer<ApproxPolicy>> sync_;
    std::vector<image_transport::Publisher> pubs_;

    image_transport::TransportHints hints_{"raw", ros::TransportHints().tcpNoDelay()};

    /* YOLO 模型 ------------------------------------------------------------------------------------------------------*/
    std::unique_ptr<trt::TensorRTRunner> model_ = nullptr;
    std::string model_path_{};
    double conf_threshold_ = 0.5;
    int batch_num_ = 4;

    std::vector<cv::Mat> raws_, perprocessed_;
    std::vector<float> scales_;
    std::vector<cv::Point> pads_;
    std::vector<std_msgs::Header> headers_;

    std::vector<float> input_;
    std::vector<float> output_;

    /* 话题配置 -------------------------------------------------------------------------------------------------------*/
    std::vector<std::string> image_topics_;
    std::vector<std::string> detected_topics_;

    /* Letterbox 参数 -------------------------------------------------------------------------------------------------*/
    cv::Scalar letterbox_color_{114, 114, 114};
    
    ros::Time last_t0_;
    bool print_debug_ = false;
};

} // namespace observer
