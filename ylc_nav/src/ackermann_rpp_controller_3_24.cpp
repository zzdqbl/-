#include "ylc_nav_rpp/ackermann_rpp_controller_3_24.hpp"
#include "nav2_core/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>
#include <string>

namespace ackermann_rpp_controller_3_24
{

void AckermannRppController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, 
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw nav2_core::PlannerException("Unable to lock node in AckermannRppController!");
  }

  plugin_name_ = name;

  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::configure(
    parent, name, tf, costmap_ros);

  RCLCPP_INFO(node_->get_logger(), "✅ AckermannRPP 双PID + 独立积分 加载完成");

  // 加载两套PID参数
  getPidParameters();
  getBackwardPidParameters();

  // 两个方向独立初始化
  integral_forward_ = 0.0;
  integral_backward_ = 0.0;
  previous_error_ = 0.0;
  last_compute_time_ = node_->now();
  pid_initialized_ = true;
}

void AckermannRppController::getPidParameters()
{
  if (!node_) {
    throw nav2_core::PlannerException("Node not initialized!");
  }

  std::string param_prefix = plugin_name_ + ".angular_pid.";

  node_->declare_parameter(param_prefix + "kp", 2.0);
  node_->declare_parameter(param_prefix + "ki", 0.1);
  node_->declare_parameter(param_prefix + "kd", 0.5);
  node_->declare_parameter(param_prefix + "output_min", -1.5);
  node_->declare_parameter(param_prefix + "output_max", 1.5);
  node_->declare_parameter(param_prefix + "offset", 0.0);
  node_->declare_parameter(param_prefix + "max_integral", 10.0);

  node_->get_parameter(param_prefix + "kp", angular_pid_config_.kp);
  node_->get_parameter(param_prefix + "ki", angular_pid_config_.ki);
  node_->get_parameter(param_prefix + "kd", angular_pid_config_.kd);
  node_->get_parameter(param_prefix + "output_min", angular_pid_config_.output_min);
  node_->get_parameter(param_prefix + "output_max", angular_pid_config_.output_max);
  node_->get_parameter(param_prefix + "offset", angular_pid_config_.offset);
  node_->get_parameter(param_prefix + "max_integral", angular_pid_config_.max_integral);
}

void AckermannRppController::getBackwardPidParameters()
{
  if (!node_) {
    throw nav2_core::PlannerException("Node not initialized!");
  }

  std::string param_prefix = plugin_name_ + ".backward_angular_pid.";

  node_->declare_parameter(param_prefix + "kp", 1.5);
  node_->declare_parameter(param_prefix + "ki", 0.05);
  node_->declare_parameter(param_prefix + "kd", 0.3);
  node_->declare_parameter(param_prefix + "output_min", -0.8);
  node_->declare_parameter(param_prefix + "output_max", 0.8);
  node_->declare_parameter(param_prefix + "offset", 0.0);
  node_->declare_parameter(param_prefix + "max_integral", 5.0);

  node_->get_parameter(param_prefix + "kp", backward_pid_config_.kp);
  node_->get_parameter(param_prefix + "ki", backward_pid_config_.ki);
  node_->get_parameter(param_prefix + "kd", backward_pid_config_.kd);
  node_->get_parameter(param_prefix + "output_min", backward_pid_config_.output_min);
  node_->get_parameter(param_prefix + "output_max", backward_pid_config_.output_max);
  node_->get_parameter(param_prefix + "offset", backward_pid_config_.offset);
  node_->get_parameter(param_prefix + "max_integral", backward_pid_config_.max_integral);
}

// ==============================================
// 核心：独立积分 PID 计算
// ==============================================
double AckermannRppController::computeAngularPid(
  double error, double dt, const PidConfig & config, double & integral)
{
  if (!pid_initialized_ || dt <= 0.0) {
    return 0.0;
  }

  double p_term = config.kp * error;

  // 积分：各自独立
  integral += error * dt;
  if (integral > config.max_integral) integral = config.max_integral;
  if (integral < -config.max_integral) integral = -config.max_integral;

  double i_term = config.ki * integral;
  double derivative = (error - previous_error_) / dt;
  double d_term = config.kd * derivative;

  previous_error_ = error;
  double output = p_term + i_term + d_term + config.offset;
  output = std::max(config.output_min, std::min(config.output_max, output));

  return output;
}

geometry_msgs::msg::TwistStamped AckermannRppController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  auto transformed_plan = transformGlobalPlan(pose);
  double lookahead_dist = getLookAheadDistance(speed);

  if (allow_reversing_) {
    double dist_to_cusp = findVelocitySignChange(transformed_plan);
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double target_x = carrot_pose.pose.position.x;
  double target_y = carrot_pose.pose.position.y;
  double error = std::atan2(target_y, target_x);

  while (error > M_PI) error -= 2 * M_PI;
  while (error < -M_PI) error += 2 * M_PI;

  rclcpp::Time now = node_->now();
  double dt = (now - last_compute_time_).seconds();
  last_compute_time_ = now;

  if (dt > 1.0 || dt <= 0.0) {
    dt = 0.01;
  }

  // 旋转到目标/路径逻辑
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    error = angle_to_goal;
  } 
  else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    error = angle_to_heading;
  }
  while (error > M_PI) error -= 2 * M_PI;
  while (error < -M_PI) error += 2 * M_PI;

  // 方向判断
  double sign = 1.0;
  if (allow_reversing_) {
    sign = target_x >= 0.0 ? 1.0 : -1.0;
  }

  double linear_vel = desired_linear_vel_;
  applyConstraints(0.0, speed, costAtPose(pose.pose.position.x, pose.pose.position.y),
    transformed_plan, linear_vel, sign);

  // ==============================================
  // 最终正确逻辑：双PID + 双积分 + 方向正确
  // ==============================================
  double angular_vel = 0.0;
  if (sign > 0.0) {
    angular_vel = computeAngularPid(error, dt, angular_pid_config_, integral_forward_);
  } else {
    angular_vel = computeAngularPid(error, dt, backward_pid_config_, integral_backward_);
    angular_vel = -angular_vel;  // 阿克曼倒车必须反向
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  return cmd_vel;
}

}  // namespace ackermann_rpp_controller_3_24

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ackermann_rpp_controller_3_24::AckermannRppController,
  nav2_core::Controller)