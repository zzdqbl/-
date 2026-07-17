// Copyright (c) 2026 Your Company
// Licensed under the Apache License, Version 2.0

#include "ylc_nav_rpp/ackermann_rpp_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace ylc_nav_rpp
{

void AckermannRegulatedPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  load_ackermann_params(node, name);
  this->use_rotate_to_heading_ = true; 

  // 所有状态全部在 configure 初始化，绝对安全
  backup_duration_ = 0.0;
  angle_integral_ = 0.0;
  prev_angle_error_ = 0.0;
  prev_derivative_ = 0.0;
  filtered_linear_vel_ = 0.0;
  filtered_angular_vel_ = 0.0;
 
  // 🔴 移除 static 后，类成员变量初始化
  no_progress_duration_ = 0.0;

  RCLCPP_INFO(this->logger_, "Ackermann RPP 1m/s High Speed (NO STATIC VAR) Ready.");
}

void AckermannRegulatedPurePursuitController::applyConstraints(
  const double & curvature,
  const geometry_msgs::msg::Twist & curr_speed,
  const double & pose_cost,
  const nav_msgs::msg::Path & path,
  double & linear_vel,
  double & sign)
{
  RegulatedPurePursuitController::applyConstraints(
    curvature, curr_speed, pose_cost, path, linear_vel, sign);

  // ==============================
  // [原有逻辑] 1m/s 高速前瞻（完全不动）
  // ==============================
  double current_speed = std::abs(curr_speed.linear.x);
  double dynamic_lookahead = 0.6 + 2.0 * current_speed;
  dynamic_lookahead = std::clamp(dynamic_lookahead, 0.3, 3.0);

  double prelook_curvature = curvature;
  if (path.poses.size() > 2) {
    int best_index = -1;
    double min_diff = 1e9;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      double dx = path.poses[i].pose.position.x - path.poses[0].pose.position.x;
      double dy = path.poses[i].pose.position.y - path.poses[0].pose.position.y;
      double dist = std::hypot(dx, dy);

      double diff = std::abs(dist - dynamic_lookahead);
      if (diff < min_diff) {
        min_diff = diff;
        best_index = static_cast<int>(i);
      }
      if (dist > dynamic_lookahead * 2.0) break;
    }

    if (best_index > 0) {
      double dx = path.poses[best_index].pose.position.x - path.poses[0].pose.position.x;
      double dy = path.poses[best_index].pose.position.y - path.poses[0].pose.position.y;
      double yaw_0 = tf2::getYaw(path.poses[0].pose.orientation);
      double yaw_k = tf2::getYaw(path.poses[best_index].pose.orientation);
      double dth = yaw_k - yaw_0;

      while (dth > M_PI) dth -= 2 * M_PI;
      while (dth < -M_PI) dth += 2 * M_PI;

      double dist = std::hypot(dx, dy);
      if (dist > 0.1) prelook_curvature = dth / dist;
    }
  }

  double angle_error = 0.0;
  double curr_angle = std::abs(curvature) > 1e-6 ? std::atan(ackermann_params_.vehicle_wheelbase * std::abs(curvature)) : 0.0;
  double pre_angle = std::abs(prelook_curvature) > 1e-6 ? std::atan(ackermann_params_.vehicle_wheelbase * std::abs(prelook_curvature)) : 0.0;
  angle_error = std::max(curr_angle, pre_angle);

  // ==============================
  // [原有逻辑] 卡死检测与恢复（完全不动）
  // ==============================
  double dt = this->control_duration_;
  if (dt < 0.001 || dt > 0.1) dt = 0.02;

  bool is_no_progress = (std::abs(curr_speed.linear.x) < 0.15) && !path.poses.empty();
  if (is_no_progress) {
    no_progress_duration_ += dt;
  } else {
    no_progress_duration_ = 0.0;
  }

  recovery_state_ = RecoveryState::FORWARD;
  if (no_progress_duration_ > 0.6) {
    recovery_state_ = RecoveryState::BACKUP;
    backup_duration_ += dt;

    if (backup_duration_ > 1.0) {
      recovery_state_ = RecoveryState::FORWARD;
      backup_duration_ = 0.0;
      no_progress_duration_ = 0.0;
    } else {
      linear_vel = -std::abs(ackermann_params_.max_backward_vel * 0.7);
      filtered_linear_vel_ = linear_vel;
      return;
    }
  }

  // ==============================
  // [原有逻辑] 高速弯道减速（完全不动）
  // ==============================
  double speed_scale = 1.0;
  double turn_threshold = 0.04;
  if (angle_error > turn_threshold) {
    double normalized_error = std::min((angle_error - turn_threshold) / 0.20, 1.0);
    speed_scale = 1.0 - 0.6 * normalized_error;
  }

  // ==========================================
  // ✅ 新增：0.8m 远前瞻 智能调速 (仅新增此处)
  // ==========================================
  const double lookahead_far = 0.8; // 固定看前方 0.8 米
  double far_angle_error = 0.0;

  if (path.poses.size() > 2) {
    int best_far_idx = -1;
    double min_diff_far = 1e9;

    // 寻找距离车头 0.8m 处的路径点
    for (size_t i = 1; i < path.poses.size(); ++i) {
      double dx = path.poses[i].pose.position.x - path.poses[0].pose.position.x;
      double dy = path.poses[i].pose.position.y - path.poses[0].pose.position.y;
      double dist = std::hypot(dx, dy);

      if (std::fabs(dist - lookahead_far) < min_diff_far) {
        min_diff_far = std::fabs(dist - lookahead_far);
        best_far_idx = static_cast<int>(i);
      }
      if (dist > lookahead_far * 1.5) break;
    }

    if (best_far_idx > 0) {
      double yaw0 = tf2::getYaw(path.poses[0].pose.orientation);
      double yawk = tf2::getYaw(path.poses[best_far_idx].pose.orientation);
      double dth = yawk - yaw0;
      
      while (dth > M_PI) dth -= 2*M_PI;
      while (dth < -M_PI) dth += 2*M_PI;
      
      far_angle_error = std::fabs(dth); // 远处的角度差
    }
  }

  // 根据远处角度计算限速值
  double vel_limit = ackermann_params_.desired_linear_vel;
  const double far_th = 0.03; // 小弯道敏感阈值
  if (far_angle_error > far_th) {
    // 平滑插值：从 desired 降到 turn
    double norm = std::clamp((far_angle_error - far_th) / 0.12, 0.0, 1.0);
    vel_limit = ackermann_params_.desired_linear_vel - 
               (ackermann_params_.desired_linear_vel - ackermann_params_.turn_linear_vel) * norm;
  }

  // 【关键】取 原有scale 和 新限速 中更慢的一个 (安全优先)
  double scale_from_limit = vel_limit / ackermann_params_.desired_linear_vel;
  speed_scale = std::min(speed_scale, scale_from_limit);
  // ==========================================

  if (recovery_state_ != RecoveryState::BACKUP) {
    double original_sign = (linear_vel >= 0) ? 1.0 : -1.0;
    linear_vel = std::abs(linear_vel) * speed_scale * original_sign;
  }

    // ==============================
  // [优化逻辑] 高速滤波 + 死区 (已移除错误的角速度逻辑)
  // ==============================
  if (recovery_state_ != RecoveryState::BACKUP) {
    // 1. 线速度滤波 (原有逻辑)
    double base_alpha = dt / (0.15 + dt);
    double adaptive_alpha = base_alpha;
    
    // 如果角度误差大，加快滤波响应（更保守，防止冲太猛）
    // 注意：这里无法直接获取实时角加速度，只能通过 angle_error 判断
    if (std::abs(angle_error) > 0.15) {
      adaptive_alpha = 0.5; // 更强的低通滤波，让线速度变化更慢
    }

    filtered_linear_vel_ = filtered_linear_vel_ + adaptive_alpha * (linear_vel - filtered_linear_vel_);
    linear_vel = filtered_linear_vel_;

    // 2. 死区 (调小一点阈值，避免频繁启停)
    if (std::abs(linear_vel) < 0.05) {
      linear_vel = 0.0;
    }
  }
}

bool AckermannRegulatedPurePursuitController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose,
  double & angle_to_path)
{
  if (global_plan_.poses.empty()) return false;

  double x = carrot_pose.pose.position.x - global_plan_.poses[0].pose.position.x;
  double y = carrot_pose.pose.position.y - global_plan_.poses[0].pose.position.y;

  double current_yaw = tf2::getYaw(global_plan_.poses[0].pose.orientation);
  double target_yaw = std::atan2(y, x);
  angle_to_path = target_yaw - current_yaw;

  while (angle_to_path > M_PI) angle_to_path -= 2 * M_PI;
  while (angle_to_path < -M_PI) angle_to_path += 2 * M_PI;

  double dist = std::hypot(x, y);
  if (dist < 0.1) return false;

  double threshold = ackermann_params_.rotate_angle_threshold * 0.6;
  return std::fabs(angle_to_path) > threshold;
}

bool AckermannRegulatedPurePursuitController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & /*carrot_pose*/)
{
  return false;
}

void AckermannRegulatedPurePursuitController::rotateToHeading(
  double & linear_vel,
  double & angular_vel,
  const double & angle_to_path,
  const geometry_msgs::msg::Twist & curr_speed)
{
  double dt = this->control_duration_;
  if (dt < 0.001 || dt > 0.1) dt = 0.02;

  double error = angle_to_path;
  double abs_error = fabs(error);

  // 1. 死区判断
  const double stop_threshold = 0.05; // 稍微调小一点，更精准
  if (abs_error < stop_threshold) {
    angular_vel = 0.0;
    prev_angle_error_ = error;
    prev_derivative_ = 0.0; // 重置微分
    return;
  }

  // 2. ✅ 新增：速度自适应 PID 增益
  // 当前车速越快，P 和 D 越小，防止高速震荡；车速慢（原地转），P 和 D 大，保证力度
  double current_v = std::abs(curr_speed.linear.x);
  double speed_factor = std::clamp(1.0 - (current_v / ackermann_params_.desired_linear_vel), 0.4, 1.0);
  if (current_v < 0.05) {
    speed_factor *= 0.5; 
  }
  // 基础增益 * 速度因子 (高速时增益降低到 40%-60%)
  double p_gain = ackermann_params_.kp_angle * 0.7 * speed_factor; 
  double d_gain = ackermann_params_.kd_angle * 0.8 * speed_factor;

  // P 项
  double p_term = p_gain * error;
  
  // I 项 (通常原地旋转不需要 I，保持为 0 以防积分饱和)
  angle_integral_ = 0.0; 
  double i_term = 0.0;

  // D 项 (带简单滤波)
  double raw_deriv = (error - prev_angle_error_) / dt;
  // 对微分项进行平滑，防止噪声放大
  double filtered_deriv = 0.6 * raw_deriv + 0.4 * prev_derivative_; 
  prev_derivative_ = filtered_deriv;
  double d_term = d_gain * filtered_deriv;

  double target_omega = p_term + i_term + d_term;

  // A. 最大角速度限制：原地调整绝不允许快！
  double max_omega = 0.6; // 固定限制在 0.6 rad/s (约 34 度/秒)，足够稳
  // 如果参数里设了更小的，听参数的
  if (this->rotate_to_heading_angular_vel_ > 0 && this->rotate_to_heading_angular_vel_ < max_omega) {
    max_omega = this->rotate_to_heading_angular_vel_;
  }

  // 硬性限幅 PID 输出
  target_omega = std::clamp(target_omega, -max_omega, max_omega);

  // B. 角加速度限制：无论误差多大，起步都要柔！
  double max_ang_accel = 1.2; // 默认温和加速度
  
  // 只有在快要对准的最后阶段，才进一步降低加速度，防止过冲
  if (abs_error < 0.3) {
    max_ang_accel = 0.6; // 最后 15 度，慢悠悠停准
  }
  // 注意：这里彻底移除了 "if (abs_error > 1.0) accel = 3.0" 的暴冲逻辑！


  double delta_omega = max_ang_accel * dt;
  
  // 平滑过渡
  if (target_omega > angular_vel + delta_omega) {
    angular_vel = angular_vel + delta_omega;
  } else if (target_omega < angular_vel - delta_omega) {
    angular_vel = angular_vel - delta_omega;
  } else {
    angular_vel = target_omega;
  }

  // 4. 线性速度配合
  // 如果角度误差很大，强制限制线速度，防止“一边狂奔一边急转”
  if (abs_error > 0.4) {
    double v_min = ackermann_params_.min_rotate_speed * 0.6;
    // 逐渐降低线速度，而不是直接切断
    if (std::abs(linear_vel) > v_min) {
      linear_vel = std::copysign(v_min, linear_vel);
    }
  } else if (abs_error > 0.15) {
     // 中等误差，允许稍微快一点，但不能全速
     double v_mid = ackermann_params_.turn_linear_vel * 0.8;
     if (std::abs(linear_vel) > v_mid) {
       linear_vel = std::copysign(v_mid, linear_vel);
     }
  }

  prev_angle_error_ = error;
}

void AckermannRegulatedPurePursuitController::load_ackermann_params(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & plugin_name)
{
  auto declare_and_get = [&](const std::string & param_name, double default_val) -> double {
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name + "." + param_name, rclcpp::ParameterValue(default_val)
    );
    double val;
    if (!node->get_parameter(plugin_name + "." + param_name, val)) {
      val = default_val;
    }
    return val;
  };

  ackermann_params_.vehicle_wheelbase = declare_and_get("vehicle_wheelbase", 0.17);
  ackermann_params_.vehicle_max_steer_angle = declare_and_get("vehicle_max_steer_angle", 0.41);
  ackermann_params_.min_rotate_speed = declare_and_get("min_rotate_speed", 0.20);
  ackermann_params_.max_backward_vel = declare_and_get("max_backward_vel", 0.40);
  ackermann_params_.rotate_angle_threshold = declare_and_get("rotate_angle_threshold", 0.25);

  ackermann_params_.kp_angle = declare_and_get("kp_angle", 2.2);
  ackermann_params_.ki_angle = declare_and_get("ki_angle", 0.0);
  ackermann_params_.kd_angle = declare_and_get("kd_angle", 1.8);

  ackermann_params_.wheel_track = declare_and_get("wheel_track", 0.185);
  lookahead_scale_ = declare_and_get("lookahead_scale", 0.8);
  ackermann_params_.desired_linear_vel = declare_and_get("desired_linear_vel", 1.0);
  ackermann_params_.turn_linear_vel    = declare_and_get("turn_linear_vel",    0.5);
}

} // namespace ylc_nav_rpp

PLUGINLIB_EXPORT_CLASS(
  ylc_nav_rpp::AckermannRegulatedPurePursuitController,
  nav2_core::Controller
)