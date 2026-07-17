// ackermann_rpp_controller_3_24.hpp

#ifndef ACKERMANN_RPP_CONTROLLER_3_24_HPP_
#define ACKERMANN_RPP_CONTROLLER_3_24_HPP_

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"

#include <string>
#include <memory>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace ackermann_rpp_controller_3_24
{

class AckermannRppController : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  AckermannRppController() = default;
  ~AckermannRppController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override;

private:
  struct PidConfig {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double output_min = -1.0;
    double output_max = 1.0;
    double offset = 0.0;
    double max_integral = 10.0;
  };

  PidConfig angular_pid_config_;
  PidConfig backward_pid_config_;   // 后退PID（新增）
  // 新增：保存节点指针
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  
  // 新增：保存插件名称，避免依赖父类可能不存在的 name_
  std::string plugin_name_;

  double integral_error_ = 0.0;
  double integral_forward_ = 0.0;
  double integral_backward_ = 0.0;
  double previous_error_ = 0.0;
  rclcpp::Time last_compute_time_;
  bool pid_initialized_ = false;

  void getPidParameters();
  void getBackwardPidParameters();  // 新增
  double computeAngularPid(double error, double dt, const PidConfig & config, double & integral); // ✅ 正确
};

}  // namespace ackermann_rpp_controller_3_24

#endif  // ACKERMANN_RPP_CONTROLLER_3_24_HPP_