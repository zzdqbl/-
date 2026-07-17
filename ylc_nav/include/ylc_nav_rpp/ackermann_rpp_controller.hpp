// Copyright (c) 2026 Your Company
// Licensed under the Apache License, Version 2.0

#ifndef YLC_NAV_RPP__ACKERMANN_RPP_CONTROLLER_HPP_
#define YLC_NAV_RPP__ACKERMANN_RPP_CONTROLLER_HPP_

// 官方头文件
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

// ROS2核心依赖
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // ✅ 补充必要头文件

namespace ylc_nav_rpp
{
// 恢复状态枚举（作用域内可见）
enum class RecoveryState {
  FORWARD = 0,    // 正常前进
  BACKUP = 1,     // 倒车
  RECOVERY = 2    // 恢复转向
};

// 阿克曼参数结构体（封装所有自定义参数）
struct AckermannParams
{
  // 基础硬件参数
  double vehicle_wheelbase = 0.3;          // 轴距 (m)
  double vehicle_max_steer_angle = 0.785;  // 最大转向角 (rad，≈45°)
  double wheel_track = 0.2;                // 轮距 (m)
  
  // 运动控制参数
  double min_rotate_speed = 0.25;          // 最小转向线速度 (m/s)
  double max_backward_vel = 0.2;           // 最大倒车速度 (m/s)
  double rotate_angle_threshold = 0.5;     // 旋转触发角度阈值 (rad)
  
  double desired_linear_vel;   // 直线速度 (YAML 配置)
  double turn_linear_vel;      // 转弯速度 (YAML 配置)
  // PID参数
  double kp_angle = 2.0;
  double ki_angle = 0.0;
  double kd_angle = 0.5;
};

class AckermannRegulatedPurePursuitController 
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  // 构造/析构函数（显式默认，符合C++11规范）
  AckermannRegulatedPurePursuitController() = default;
  ~AckermannRegulatedPurePursuitController() override = default;

  // ✅ 核心修正1：添加override关键字，匹配父类虚函数
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) ;

  void applyConstraints(
    const double & curvature,
    const geometry_msgs::msg::Twist & curr_speed,
    const double & pose_cost,
    const nav_msgs::msg::Path & path,
    double & linear_vel,
    double & sign) ;

protected:
  // ✅ 核心修正2：所有重写的protected函数添加override
  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose, 
    double & angle_to_path) ;

  bool shouldRotateToGoalHeading(
    const geometry_msgs::msg::PoseStamped & carrot_pose) ;

  void rotateToHeading(
    double & linear_vel, 
    double & angular_vel,
    const double & angle_to_path, 
    const geometry_msgs::msg::Twist & curr_speed) ;

private:
  // 阿克曼参数（私有化，仅类内访问）
  AckermannParams ackermann_params_;
  
  // PID状态变量（旋转控制专用）
  double angle_integral_ = 0.0;
  double prev_angle_error_ = 0.0;
  double lookahead_scale_ = 0.8;       // 前瞻距离缩放系数
  double prev_derivative_ = 0.0;       // 上一次微分值 (用于低通滤波)
  RecoveryState recovery_state_ = RecoveryState::FORWARD; // ✅ 初始化默认状态
  double backup_duration_ = 0.0;                         // ✅ 初始化倒车时长
  double filtered_linear_vel_ = 0.0;                     // 速度滤波值
  double filtered_angular_vel_ = 0.0;                    // 角速度滤波值
  //int backup_cycle_count_ = 0;                           // ✅ 初始化倒车周期计数
  double no_progress_duration_ = 0.0; 
  // 辅助函数：加载阿克曼参数（私有化，避免外部调用）
  void load_ackermann_params(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & plugin_name);
};

}  // namespace ylc_nav_rpp

#endif  // YLC_NAV_RPP__ACKERMANN_RPP_CONTROLLER_HPP_