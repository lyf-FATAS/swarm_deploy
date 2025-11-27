/**
 * @file    drone_control_node.hpp
 * @brief   无人机策略控制（ROS1 节点）
 * @date    2025-08-09
 */

#pragma once

#include "TensorRTRunner.hpp"
#include "ObservationQueue.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32MultiArray.h>

#include <tf2_ros/transform_broadcaster.h>

#include "quadrotor_msgs/PositionCommand.h"

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/Marker.h>

#include <array>
#include <boost/bind.hpp>
#include <atomic>
#include <cmath>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>
#include <chrono>

using SteadyClock = std::chrono::steady_clock;

/************************************************************************************************************************
 * @brief   无人机控制器节点类（ROS1）
 ***********************************************************************************************************************/
class DroneControllerNode
{
public:
    explicit DroneControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~DroneControllerNode();

private:
    /* 初始化 */
    void initParams(ros::NodeHandle& pnh);
    void initInference(std::string& model_path, std::vector<int>& input_shape);
    void initObservation();

    /* 回调 */
    void inferCallback(const ros::TimerEvent&);
    void controlCallback(const ros::TimerEvent&);
    void onTarget(const geometry_msgs::PointConstPtr& msg);
    void onOdom(const nav_msgs::OdometryPtr& msg);
    void onSelfMocap(const nav_msgs::Odometry::ConstPtr& msg);
    void onOtherMocap(const nav_msgs::Odometry::ConstPtr& msg, int idx);
    void onOtherDrones(const sensor_msgs::PointCloud::ConstPtr& msg);
    void onTrigger(const geometry_msgs::PoseStampedPtr& msg);

    /* 交互式 marker */
        // 初始化与反馈处理
    void initInteractiveMarkers(ros::NodeHandle& pnh);
    void createGoalMarker();
    void createObstacleMarker(int idx);
    void publishObstacles(); // 把障碍物数组发出去
        // 反馈回调
    void onGoalFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb);
    void onObstacleFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb, int idx);

    /* 工具 */
    template <typename T, size_t N>
    float norm(const std::array<T, N>& arr) const
    {
        float sum_sq = std::accumulate(
            arr.begin(), arr.end(), 0.0f,
            [](float acc, float x) { return acc + x * x; }
        );
        return std::sqrt(sum_sq);
    }

private:
    /* 推理与观测 */
    std::unique_ptr<trt::TensorRTRunner> runner_;
    std::unique_ptr<ObservationQueue>     obs_queue_;

    bool enable_ = false;

    std::array<float, 2> normalized_action_ = {};   // 上次动作（x, y，已归一化）
    std::array<float, 2> target_position_   = {};   // 目标位置（x, y）
    std::array<float, 2> position_          = {};   // 自身位置（x, y）
    std::array<float, 2> velocity_          = {};   // 自身速度（x, y）
    std::vector<float>   other_drones_obs_;         // 其他飞机观测 [other_num * (x, y, z, mask)]
    std::vector<float>   obs_data_single_;          // 单次观测
    std::vector<float>   state_;                    // 队列展平后的状态
    std::vector<float>   action_;                   // 网络输出

    /* 输出 */
    std::array<float, 2> output_acc_ = {};          // 期望加速度（x, y）
    std::array<float, 2> output_vel_ = {};          // 期望速度（x, y）
    std::array<float, 2> output_pos_ = {};          // 期望位置（x, y）

    /* 约束与周期 */
    float infer_dt_    = 0.05f;
    float control_dt_  = 0.01f;
    float action_clip_ = 1.0f;
    float max_vel_     = 1.5f;
    float max_acc_     = 6.0f;
    float altitude_    = 1.0f;

    /* 尺寸参数 */
    int drones_num_       = 5;
    int obs_history_len_  = 10;
    int obstacles_num_    = 0;
    int virtual_obstacles_num_ = 0;
    int other_drones_dim_ = 0;
    int obs_dim_          = 0;

    /* ROS */
    ros::Subscriber sub_target_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_self_mocap_;
    ros::Subscriber sub_others_;
    std::vector<ros::Subscriber> sub_others_mocap_;
    ros::Subscriber sub_trigger_;
    ros::Publisher  pub_control_;
    ros::Publisher  pub_action_;                    // 可选：发布网络原始动作
    ros::Timer      timer_inference_;
    ros::Timer      timer_control_process_;
    tf2_ros::TransformBroadcaster br_;

    nav_msgs::Odometry odom_msg_;

    /* 配置 */
    bool  print_debug_   = true;
    int   warmup_iters_  = 200;
    std::string model_path_;
    std::string self_mocap_topic_;
    std::vector<std::string> other_mocap_topics_;
    bool use_mocap_for_others_{true};

    /* 线程同步 */
    std::mutex mtx_;

    /* 时间统计 */
    SteadyClock::time_point last_t0_;

private:
    // === Interactive Markers ===
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> im_server_;
    std::string world_frame_{"world"};
    
    bool use_obstacles_as_others_{true};

    struct Obstacle {
        float x{0.f}, y{0.f}, z{0.f};
        bool mask{true};
    };
    std::vector<Obstacle> obstacles_;

    // 发布障碍物数组（[x, y, z, mask]*）
    ros::Publisher pub_obstacles_;
    ros::Publisher pub_target_marker_;
    std::string obstacles_topic_{"obstacles"};
};
