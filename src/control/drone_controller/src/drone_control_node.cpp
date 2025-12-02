/**
 * @file    drone_control_node.cpp
 * @brief   无人机策略控制（ROS1 节点实现）
 * @date    2025-08-09
 */

#include "drone_control_node.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // tf2::fromMsg

using ms_f = std::chrono::duration<double, std::milli>;

/* ====================================== 构造/析构 ====================================== */

DroneControllerNode::DroneControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: last_t0_(SteadyClock::now())
{
    initParams(pnh);

    /* 观测初始化 */
    initObservation();

    /* TensorRT 初始化 */
    std::vector<int> input_shape = {obs_history_len_ * obs_dim_};
    initInference(model_path_, input_shape);

    /* 交互式 marker（RViz） */
    im_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "drone_control_im", "", false  // topic ns, server id, spin_thread
    );
    initInteractiveMarkers(pnh);
    im_server_->applyChanges();
    pub_target_marker_ = nh.advertise<geometry_msgs::Point>("target_marker", 10);
    pub_obstacles_     = nh.advertise<std_msgs::Float32MultiArray>("obstacles_marker", 10);

    /* 话题（支持参数化） */
    std::string topic_target  = pnh.param<std::string>("target_topic",  "target_position");
    std::string topic_odom    = pnh.param<std::string>("odom_topic",    "/ekf/ekf_odom");
    std::string topic_others  = pnh.param<std::string>("others_topic",  "other_drones_obs");
    std::string topic_control = pnh.param<std::string>("control_topic", "position_cmd");
    std::string topic_action  = pnh.param<std::string>("action_topic",  "policy_action");

    sub_target_  = nh.subscribe(topic_target, 10, &DroneControllerNode::onTarget, this);
    if (!self_mocap_topic_.empty())
    {
        sub_self_mocap_ = nh.subscribe(self_mocap_topic_, 1, &DroneControllerNode::onSelfMocap, this, ros::TransportHints().tcpNoDelay());
        ROS_INFO_STREAM("[drone_control_node] Use mocap for self pose: " << self_mocap_topic_);
    }
    else if (!topic_odom.empty())
    {
        sub_odom_    = nh.subscribe(topic_odom,   1,  &DroneControllerNode::onOdom,   this, ros::TransportHints().tcpNoDelay());
        ROS_WARN_STREAM("[drone_control_node] self_mocap_topic empty, fallback to odom: " << topic_odom);
    }

    if (use_mocap_for_others_ && !other_mocap_topics_.empty())
    {
        sub_others_mocap_.resize(other_mocap_topics_.size());
        const std::size_t sub_num = std::min(static_cast<std::size_t>(obstacles_num_), other_mocap_topics_.size());
        for (std::size_t i = 0; i < sub_num; ++i)
        {
            sub_others_mocap_[i] = nh.subscribe<nav_msgs::Odometry>(
                other_mocap_topics_[i], 1,
                boost::bind(&DroneControllerNode::onOtherMocap, this, _1, static_cast<int>(i)));
            ROS_INFO_STREAM("[drone_control_node] Use mocap for other[" << i << "]: " << other_mocap_topics_[i]);
        }
    }
    else if (!topic_others.empty())
    {
        sub_others_  = nh.subscribe(topic_others, 1,  &DroneControllerNode::onOtherDrones, this);
        ROS_INFO_STREAM("[drone_control_node] Use pointcloud for others: " << topic_others);
    }
    else
    {
        ROS_WARN("[drone_control_node] No others input configured (mocap or pointcloud).");
    }
    sub_trigger_ = nh.subscribe("/traj_start_trigger", 10, &DroneControllerNode::onTrigger, this);

    pub_control_ = nh.advertise<quadrotor_msgs::PositionCommand>(topic_control, 10);
    pub_action_  = nh.advertise<std_msgs::Float32MultiArray>(topic_action, 10);

    odom_msg_.pose.pose.orientation.w = 1.0;

    /* 预热（使 TensorRT 引擎 ready） */
    obs_queue_->getFlattened(state_);
    for (int i = 0; i < warmup_iters_; ++i)
    {
        runner_->infer(state_.data(), action_);
    }

    /* 定时器驱动主循环 */
    timer_inference_ = nh.createTimer(ros::Duration(infer_dt_), &DroneControllerNode::inferCallback, this);
    timer_control_process_ = nh.createTimer(ros::Duration(control_dt_), &DroneControllerNode::controlCallback, this);

    ROS_INFO_STREAM("\033[32m[drone_control_node] Init completely!"
                    << ", infer_dt="   << infer_dt_
                    << ", control_dt=" << control_dt_
                    << ", obs_dim="    << obs_dim_
                    << ", history="    << obs_history_len_
                    << ", others_dim=" << other_drones_dim_ << "\033[0m");
}

DroneControllerNode::~DroneControllerNode() = default;

/* ====================================== 参数/推理初始化 ====================================== */

void DroneControllerNode::initParams(ros::NodeHandle& pnh)
{
    // 参数读取
    pnh.param("infer_dt",    infer_dt_,    infer_dt_);
    pnh.param("control_dt",  control_dt_,  control_dt_);
    pnh.param("action_clip", action_clip_, action_clip_);
    pnh.param("max_vel",     max_vel_,     max_vel_);
    pnh.param("max_acc",     max_acc_,     max_acc_);
    pnh.param("drones_num",  drones_num_,  drones_num_);
    pnh.param("virtual_obstacles_num",  virtual_obstacles_num_,  virtual_obstacles_num_);
    pnh.param("obs_history_len", obs_history_len_, obs_history_len_);
    pnh.param("print_debug", print_debug_, print_debug_);
    pnh.param("warmup_iters", warmup_iters_, warmup_iters_);
    pnh.param<std::string>("model_path", model_path_, "");
    pnh.param("use_mocap_for_others", use_mocap_for_others_, use_mocap_for_others_);
    pnh.param<std::string>("self_mocap_topic", self_mocap_topic_, "/swarm_drone_0/pose");
    pnh.getParam("other_mocap_topics", other_mocap_topics_);

    // 参数计算
    obstacles_num_    = drones_num_ - 1;
    other_drones_dim_ = 4 * obstacles_num_;     // 每个其他飞机观测 [x, y, z, mask]
    obs_dim_          = 6 + other_drones_dim_;  // [ last_action(2), target(2), self_vel(2), others(4*(n-1)) ]
}

void DroneControllerNode::initInference(std::string& model_path, std::vector<int>& input_shape)
{
    runner_ = std::make_unique<trt::TensorRTRunner>(model_path);

    auto in_dims = runner_->getInputDims();
    if (std::any_of(in_dims.begin(), in_dims.end(), [](int x){ return x < 0; }))
    {
        runner_->setInputShape(input_shape);
        in_dims = runner_->getInputDims();
    }
    auto out_dims = runner_->getOutputDims();

    ROS_INFO_STREAM("[drone_control_node] [TensorRT] Input name : " << runner_->getInputName());
    ROS_INFO_STREAM("[drone_control_node] [TensorRT] Output name: " << runner_->getOutputName());

    std::ostringstream oss;
    oss << "[TensorRT] Input dims : [";
    for (size_t i = 0; i < in_dims.size(); ++i)
    {
        oss << in_dims[i] << (i + 1 != in_dims.size() ? ", " : "");
    }
    oss << "]";
    ROS_INFO_STREAM("[drone_control_node] " << oss.str());

    std::ostringstream oss2;
    oss2 << "[TensorRT] Output dims: [";
    for (size_t i = 0; i < out_dims.size(); ++i)
    {
        oss2 << out_dims[i] << (i + 1 != out_dims.size() ? ", " : "");
    }
    oss2 << "]";
    ROS_INFO_STREAM("[drone_control_node] " << oss2.str());
}

void DroneControllerNode::initObservation()
{
    // 初始化其他飞机观测（默认在远离原点处，保证启动安全）
    other_drones_obs_.resize(static_cast<size_t>(other_drones_dim_));
    for (int i = 0; i < obstacles_num_; ++i)
    {
        other_drones_obs_[4*i + 0] = 1.0f * i; // 简单排布：0, 1.0, 2.0, ...
        other_drones_obs_[4*i + 1] = 2.0f;
        other_drones_obs_[4*i + 2] = altitude_;
        other_drones_obs_[4*i + 3] = static_cast<bool>(true);  // 默认可见
    }
    // 初始化单次观测
    obs_data_single_.resize(static_cast<size_t>(obs_dim_));
    std::copy(other_drones_obs_.begin(),  other_drones_obs_.end(),  obs_data_single_.begin() + 6);
    // 初始化观测队列
    obs_queue_ = std::make_unique<ObservationQueue>(obs_history_len_, obs_dim_);
}

void DroneControllerNode::initInteractiveMarkers(ros::NodeHandle& pnh)
{
    // 初始化障碍物数组（默认在原点附近排布，半径 obstacle_radius_）
    obstacles_.resize(std::max(0, virtual_obstacles_num_));
    auto index_bias = obstacles_num_ - virtual_obstacles_num_;
    for (int i = 0; i < virtual_obstacles_num_; ++i)
    {
        obstacles_[i].x = other_drones_obs_[4*(i+index_bias) + 0];
        obstacles_[i].y = other_drones_obs_[4*(i+index_bias) + 1];
        obstacles_[i].z = other_drones_obs_[4*(i+index_bias) + 2];
        obstacles_[i].mask = true;
    }

    // 创建 goal、obstacle 交互式 marker
    createGoalMarker();
    for (int i = 0; i < virtual_obstacles_num_; ++i)
        createObstacleMarker(i);

    // 初始发布一次障碍物数组（方便其他模块）
    publishObstacles();
}

void DroneControllerNode::createGoalMarker()
{
    visualization_msgs::InteractiveMarker im;
    im.header.frame_id = world_frame_;
    im.name = "goal";
    im.description = "Goal (drag in XY)";
    im.scale = 0.3f;

    // 初始位姿：使用当前 target_position_
    {
        std::lock_guard<std::mutex> lk(mtx_);
        im.pose.position.x = target_position_[0];
        im.pose.position.y = target_position_[1];
        im.pose.position.z = altitude_;
    }

    // 形状（绿色立方体）
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::CUBE;
    m.scale.x = m.scale.y = m.scale.z = 0.1;
    m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8;

    visualization_msgs::InteractiveMarkerControl vis;
    vis.always_visible = true;
    vis.markers.push_back(m);
    im.controls.push_back(vis);

    // 平面移动（XY）
    visualization_msgs::InteractiveMarkerControl move;
    move.name = "move_xy";
    move.orientation.w = 1.0;
    move.orientation.x = 0.0;
    move.orientation.y = 1.0;
    move.orientation.z = 0.0;
    move.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    im.controls.push_back(move);

    im_server_->insert(im, boost::bind(&DroneControllerNode::onGoalFeedback, this, _1));
}

void DroneControllerNode::createObstacleMarker(int idx)
{
    visualization_msgs::InteractiveMarker im;
    im.header.frame_id = world_frame_;
    im.name = "obstacle_" + std::to_string(idx);
    im.description = "Obstacle " + std::to_string(idx);
    im.scale = 0.8f;

    {
        std::lock_guard<std::mutex> lk(mtx_);
        im.pose.position.x = obstacles_[idx].x;
        im.pose.position.y = obstacles_[idx].y;
        im.pose.position.z = altitude_;
    }

    // 形状（红色圆柱，半径 = r，高度固定为 scale）
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::CYLINDER;
    double d = 0.5;
    m.scale.x = d;
    m.scale.y = d;
    m.scale.z = 0.05;
    m.color.r = 1.0; m.color.g = 0.1; m.color.b = 0.1; m.color.a = 0.85;

    visualization_msgs::InteractiveMarkerControl vis;
    vis.always_visible = true;
    vis.markers.push_back(m);
    im.controls.push_back(vis);

    visualization_msgs::InteractiveMarkerControl move;
    move.name = "move_xy";
    move.orientation.w = 1.0;
    move.orientation.x = 0.0;
    move.orientation.y = 1.0;
    move.orientation.z = 0.0;
    move.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    im.controls.push_back(move);

    im_server_->insert(im, boost::bind(&DroneControllerNode::onObstacleFeedback, this, _1, idx));
}

void DroneControllerNode::onGoalFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb)
{
    if (fb->event_type != visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
        return;

    geometry_msgs::Point p;
    p.x = fb->pose.position.x;
    p.y = fb->pose.position.y;
    p.z = altitude_;

    {
        std::lock_guard<std::mutex> lk(mtx_);
        target_position_[0] = static_cast<float>(p.x);
        target_position_[1] = static_cast<float>(p.y);
    }

    // 同步发出（让系统里其他节点也能收到）
    geometry_msgs::Point msg = p;
    pub_target_marker_.publish(msg);
}

void DroneControllerNode::onObstacleFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb, int idx)
{
    if (idx < 0 || idx >= virtual_obstacles_num_) return;
    if (fb->event_type != visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
        return;

    {
        std::lock_guard<std::mutex> lk(mtx_);
        obstacles_[idx].x = static_cast<float>(fb->pose.position.x);
        obstacles_[idx].y = static_cast<float>(fb->pose.position.y);

        // 如果启用，把交互式障碍物写入 other_drones_obs_
        if (use_obstacles_as_others_)
        {
            auto index_bias = obstacles_num_ - virtual_obstacles_num_;
            for (int i = 0; i < virtual_obstacles_num_; ++i)
            {
                other_drones_obs_[4*(i+index_bias) + 0] = obstacles_[i].x;
                other_drones_obs_[4*(i+index_bias) + 1] = obstacles_[i].y;
                other_drones_obs_[4*(i+index_bias) + 2] = obstacles_[i].z;
                other_drones_obs_[4*(i+index_bias) + 3] = static_cast<float>(true);
            }
        }
    }

    publishObstacles();
}

void DroneControllerNode::publishObstacles()
{
    std_msgs::Float32MultiArray arr;
    arr.data.reserve(static_cast<size_t>(3 * virtual_obstacles_num_));
    {
        std::lock_guard<std::mutex> lk(mtx_);
        for (int i = 0; i < virtual_obstacles_num_; ++i)
        {
            arr.data.push_back(obstacles_[i].x);
            arr.data.push_back(obstacles_[i].y);
            arr.data.push_back(obstacles_[i].z);
            arr.data.push_back(static_cast<float>(obstacles_[i].mask));
        }
    }
    pub_obstacles_.publish(arr);
}

/* ====================================== 订阅回调 ====================================== */

void DroneControllerNode::onTarget(const geometry_msgs::PointConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    target_position_[0] = static_cast<float>(msg->x);
    target_position_[1] = static_cast<float>(msg->y);
}

void DroneControllerNode::onOdom(const nav_msgs::OdometryPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    // 消息保存
    odom_msg_ = *msg;
    // 位置信息
    position_[0] = static_cast<float>(msg->pose.pose.position.x);
    position_[1] = static_cast<float>(msg->pose.pose.position.y);
    // 速度信息
    velocity_[0] = static_cast<float>(msg->twist.twist.linear.x);
    velocity_[1] = static_cast<float>(msg->twist.twist.linear.y);

    // tf 发布
    geometry_msgs::TransformStamped t;
    t.header.stamp = msg->header.stamp;         // 用原时间戳
    t.header.frame_id = msg->header.frame_id;   // 父（如 "world"）
    t.child_frame_id  = "base_link";            // 子（如 "base_link"）

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation      = msg->pose.pose.orientation; // 直接转给 TF

    br_.sendTransform(t);
}

void DroneControllerNode::onSelfMocap(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    odom_msg_ = *msg;

    position_[0] = static_cast<float>(msg->pose.pose.position.x);
    position_[1] = static_cast<float>(msg->pose.pose.position.y);
    velocity_[0] = static_cast<float>(msg->twist.twist.linear.x);
    velocity_[1] = static_cast<float>(msg->twist.twist.linear.y);

    geometry_msgs::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = msg->header.frame_id;
    t.child_frame_id  = "base_link";

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation      = msg->pose.pose.orientation;

    br_.sendTransform(t);
}

void DroneControllerNode::onOtherDrones(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    const std::size_t n = msg->points.size();
    const std::size_t real_obstacles_num = obstacles_num_ - virtual_obstacles_num_;
    const std::size_t input_obstacles_num = std::min(real_obstacles_num, n);
    if (n >= real_obstacles_num)
    {
        ROS_WARN_THROTTLE(2.0, "[drone_control_node] real_drones_obs 过多，期望 %ld，收到 %ld", real_obstacles_num, n);
    }
    for(size_t i = 0; i < input_obstacles_num; ++i)
    {
        tf2::Quaternion q;
        // 取四元数与平移
        tf2::fromMsg(odom_msg_.pose.pose.orientation, q);
        q.normalize();  // 保险：归一化
        // 旋转到 world 下
        const tf2::Vector3 v_child(msg->points[i].x, msg->points[i].y, msg->points[i].z);
        const tf2::Vector3 v_parent = tf2::quatRotate(q, v_child);
        // 填充
        other_drones_obs_[4*i + 0] = v_parent.x();
        other_drones_obs_[4*i + 1] = v_parent.y();
        // other_drones_obs_[4*i + 2] = v_parent.z();
        other_drones_obs_[4*i + 2] = 0.0;
        other_drones_obs_[4*i + 3] = static_cast<float>(true);
    }
    for(size_t i = input_obstacles_num; i < real_obstacles_num; ++i)
    {
        other_drones_obs_[4*i + 0] = 0.;
        other_drones_obs_[4*i + 1] = 0.;
        other_drones_obs_[4*i + 2] = 0.;
        other_drones_obs_[4*i + 3] = static_cast<float>(false);
    }
}

void DroneControllerNode::onOtherMocap(const nav_msgs::Odometry::ConstPtr& msg, int idx)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (idx < 0 || idx >= obstacles_num_)
        return;

    tf2::Quaternion q_self;
    tf2::fromMsg(odom_msg_.pose.pose.orientation, q_self);
    q_self.normalize();

    tf2::Vector3 rel_world(
        msg->pose.pose.position.x - odom_msg_.pose.pose.position.x,
        msg->pose.pose.position.y - odom_msg_.pose.pose.position.y,
        msg->pose.pose.position.z - odom_msg_.pose.pose.position.z
    );

    const tf2::Vector3 rel_body = tf2::quatRotate(q_self.inverse(), rel_world);

    other_drones_obs_[4*idx + 0] = static_cast<float>(rel_body.x());
    other_drones_obs_[4*idx + 1] = static_cast<float>(rel_body.y());
    other_drones_obs_[4*idx + 2] = static_cast<float>(rel_body.z());
    other_drones_obs_[4*idx + 3] = static_cast<float>(true);
}

void DroneControllerNode::onTrigger(const geometry_msgs::PoseStampedPtr& msg)
{
    enable_ = true;
}

/* ====================================== 控制/推理主循环 ====================================== */

void DroneControllerNode::inferCallback(const ros::TimerEvent&)
{
    static bool once = false;
    if (enable_ == false)
    {
        ROS_INFO_STREAM("[drone_control_node] Waitting for trigger");
        return;
    }
    if (once == false)
    {
        once = true;
        ROS_INFO_STREAM("\033[32m[drone_control_node] Allow fly!!!! Run into inference\033[0m");
    }

    auto t0 = SteadyClock::now();

    /* 1. 组装单次观测 */
    {
        std::lock_guard<std::mutex> lock(mtx_);
        // 上一次 action
        int bias = 0;
        std::copy(normalized_action_.begin(), normalized_action_.end(), obs_data_single_.begin() + bias);
        bias += static_cast<int>(normalized_action_.size());
        // 目标位置（相对）
        obs_data_single_[bias + 0] = target_position_[0] - position_[0]; // 相对位置
        obs_data_single_[bias + 1] = target_position_[1] - position_[1]; // 相对位置
        bias += static_cast<int>(target_position_.size());
        // 自身速度
        std::copy(velocity_.begin(), velocity_.end(), obs_data_single_.begin() + bias);
        bias += static_cast<int>(velocity_.size());
        // 其他无人机观测（相对）
        int real_obstacles_num = obstacles_num_ - virtual_obstacles_num_;
        for (int i = 0; i < real_obstacles_num; ++i)
        {
            obs_data_single_[bias + 4*i + 0] = other_drones_obs_[4*i + 0];
            obs_data_single_[bias + 4*i + 1] = other_drones_obs_[4*i + 1];
            obs_data_single_[bias + 4*i + 2] = other_drones_obs_[4*i + 2];
            obs_data_single_[bias + 4*i + 3] = other_drones_obs_[4*i + 3];
        }
        for (int i = real_obstacles_num; i < obstacles_num_; ++i)
        {
            obs_data_single_[bias + 4*i + 0] = other_drones_obs_[4*i + 0] - position_[0];
            obs_data_single_[bias + 4*i + 1] = other_drones_obs_[4*i + 1] - position_[1];
            obs_data_single_[bias + 4*i + 2] = other_drones_obs_[4*i + 2] - altitude_;
            obs_data_single_[bias + 4*i + 3] = other_drones_obs_[4*i + 3];
        }
    }

    /* 2. 观测队列更新 */
    obs_queue_->push(obs_data_single_);
    if (!obs_queue_->ready())
    {
        // 队列未满，不推理
        return;
    }

    /* 3. 推理 */
    obs_queue_->getFlattened(state_);
    auto t1 = SteadyClock::now();
    runner_->infer(state_.data(), action_);
    auto t2 = SteadyClock::now();

    if (action_.size() != 2)
    {
        ROS_ERROR_THROTTLE(2.0, "[drone_control_node] Inferance output dims=%zu, expected=2", action_.size());
        return;
    }

    /* 4. 动作剪裁 */
    {
        std::lock_guard<std::mutex> lock(mtx_);
        for (size_t i = 0; i < action_.size(); ++i)
        {
            normalized_action_[i] = std::clamp(action_[i], -action_clip_, action_clip_) / action_clip_;
            output_acc_[i] = normalized_action_[i] * max_acc_;
        }
        // acc 按模长裁剪
        {
            float s = std::max(norm(output_acc_) / max_acc_, 1.0f);
            for (size_t i = 0; i < output_acc_.size(); ++i)
            {
                output_acc_[i] /= s;
            }
        }
    }

    /* 5. 调试与时间统计 */
    auto t3 = SteadyClock::now();
    const double infer_time    = std::chrono::duration_cast<ms_f>(t2 - t1).count();
    const double callback_time = std::chrono::duration_cast<ms_f>(t3 - t0).count();
    const double callback_freq = 1000.0 / std::chrono::duration_cast<ms_f>(t0 - last_t0_).count();
    last_t0_ = t0;

    if (print_debug_)
    {
        // // 适度输出：使用 ROS_*_THROTTLE 避免刷屏
        // ROS_INFO_THROTTLE(1.0,
        //     "[drone_control_node] infer=%.3f ms, cb=%.3f ms, freq=%.1f Hz | acc(%.2f,%.2f) vel(%.2f,%.2f) pos(%.2f,%.2f)",
        //     infer_time, callback_time, callback_freq,
        //     output_acc_[0], output_acc_[1],
        //     output_vel_[0], output_vel_[1],
        //     output_pos_[0], output_pos_[1]);

        std::cout << "============ Single Observation ============\n";
        std::cout << "[0] ";
        std::cout << std::fixed << std::setprecision(2);
        for (size_t i = 0; i < 22; ++i)
        {
            if (obs_data_single_[i] >= 0) std::cout << " ";
            std::cout << obs_data_single_[i] << " ";
        }
        std::cout << "\n\n";
        std::cout << "========= Current Observation Queue =========\n";
        int obs_history_len = 10;
        int obs_dim = 22;
        for (size_t i = 0; i < obs_history_len; ++i)
        {
            std::cout << "[" << i << "] ";
            std::cout << std::fixed << std::setprecision(2);
            for (size_t j = 0; j < obs_dim; ++j)
            {
                if (state_[i*obs_dim + j] >= 0) std::cout << " ";
                std::cout << state_[i*obs_dim + j] << " ";
                if (j == 1 || j == 3 || j == 5 || j == 9 || j == 13 || j == 17) std::cout << "| "; // 分隔符
            }
            std::cout << "\n";
        }
        std::cout << "\n";
        std::cout << "================ Time Consume ===============\n"
                << std::fixed << std::setprecision(3)
                << "Infer time   : " << infer_time    << " ms\n"
                << "Callback time: " << callback_time << " ms\n"
                << "Callback freq: " << callback_freq << " Hz\n"
                << "==============================================\n\n\n";
    }
}

void DroneControllerNode::controlCallback(const ros::TimerEvent&)
{
    static bool once = false;
    if (enable_ == false)
    {
        return;
    }
    if (once == false)
    {
        once = true;
        ROS_INFO_STREAM("\033[32m[drone_control_node] Allow fly!!!! Run into inference\033[0m");
    }

    /* 1. 运动学剪裁/积分 */
    std::lock_guard<std::mutex> lock(mtx_);
    std::array<float, 2> pre_vel = output_vel_;
    // vel 计算
    {
        float s = std::max(norm(output_acc_) / max_acc_, 1.0f);
        for (size_t i = 0; i < output_acc_.size(); ++i)
        {
            output_vel_[i] += output_acc_[i] * control_dt_;
        }
    }
    // vel 按模长裁剪
    {
        float s = std::max(norm(output_vel_) / max_vel_, 1.0f);
        for (size_t i = 0; i < output_vel_.size(); ++i)
        {
            output_vel_[i] /= s;
            output_acc_[i] = (output_vel_[i] - pre_vel[i]) / control_dt_;
        }
    }
    // pos 积分
    for (size_t i = 0; i < output_pos_.size(); ++i)
    {
        output_pos_[i] += (output_vel_[i] * control_dt_ + output_acc_[i] * control_dt_ * control_dt_ * 0.5f);
    }

    /* 2. 发布消息 */
    quadrotor_msgs::PositionCommand control_msg;
    control_msg.header.stamp = ros::Time::now();
    control_msg.position.x = output_pos_[0];
    control_msg.position.y = output_pos_[1];
    control_msg.position.z = altitude_;     // 固定高度
    control_msg.velocity.x = output_vel_[0];
    control_msg.velocity.y = output_vel_[1];
    control_msg.velocity.z = 0.0;
    control_msg.acceleration.x = output_acc_[0];
    control_msg.acceleration.y = output_acc_[1];
    control_msg.acceleration.z = 0.0;
    control_msg.jerk.x = 0.0;               // 固定加加速度
    control_msg.jerk.y = 0.0;               // 固定加加速度
    control_msg.jerk.z = 0.0;               // 固定加加速度
    control_msg.yaw = 0.0;                  // 固定航向
    control_msg.yaw_dot = 0.0;              // 固定航向速度
    pub_control_.publish(control_msg);

    std_msgs::Float32MultiArray act_msg;
    act_msg.data.assign(action_.begin(), action_.end());
    pub_action_.publish(act_msg);
}

/* ====================================== main ====================================== */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    DroneControllerNode node(nh, pnh);
    ros::spin();
    return 0;
}
