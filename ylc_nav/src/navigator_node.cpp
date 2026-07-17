/**
 * @file navigator_node.cpp
 * @brief Yiliaoche Navigation Core Node
 * @author Yiliaoche Team
 * @version 0.0.1
 * @date 2026-02-22
 * 
 * @copyright Copyright (c) 2026 Yiliaoche Team
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
using namespace std::chrono_literals;

/**
 * @class NavigatorNode
 * @brief Main navigation node for Yiliaoche
 * 
 * This node handles navigation goals and waypoint following
 * using Nav2 action servers.
 */
class NavigatorNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for NavigatorNode
   */
  NavigatorNode()
  : Node("yiliaoche_navigator")
  {
    RCLCPP_INFO(this->get_logger(), "=================================");
    RCLCPP_INFO(this->get_logger(), "Yiliaoche Navigator Node Started");
    RCLCPP_INFO(this->get_logger(), "=================================");

    // Create NavigateToPose action client
    navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");

    // Create FollowWaypoints action client
    follow_waypoints_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
      this, "follow_waypoints");

    // Wait for action servers with timeout
    this->wait_for_servers();

    // 可选：取消demo定时器，避免自动发测试目标
    // timer_ = this->create_wall_timer(10s, std::bind(&NavigatorNode::send_demo_goal, this));
  }

  /**
   * @brief Send a single navigation goal
   * @param x X coordinate in map frame
   * @param y Y coordinate in map frame
   * @param theta Orientation (radians)
   */
  void send_goal(double x, double y, double theta = 0.0)
  {
    if (!navigate_to_pose_client_->action_server_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not ready!");
      return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    
    // Convert theta to quaternion
    goal_msg.pose.pose.orientation.z = sin(theta / 2.0);
    goal_msg.pose.pose.orientation.w = cos(theta / 2.0);

    RCLCPP_INFO(this->get_logger(), "Sending goal: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
      std::bind(&NavigatorNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = 
      std::bind(&NavigatorNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = 
      std::bind(&NavigatorNode::result_callback, this, std::placeholders::_1);

    navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Cancel current navigation goal
   */
  void cancel_goal()
  {
    if (current_goal_handle_) {
      RCLCPP_INFO(this->get_logger(), "Canceling current goal...");
      navigate_to_pose_client_->async_cancel_goal(current_goal_handle_);
    }
  }

private:
  /**
   * @brief Wait for Nav2 action servers to be ready
   */
  void wait_for_servers()
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action servers...");
    
    bool nav_ready = navigate_to_pose_client_->wait_for_action_server(30s);
    bool wp_ready = follow_waypoints_client_->wait_for_action_server(30s);

    if (nav_ready && wp_ready) {
      RCLCPP_INFO(this->get_logger(), "All Nav2 action servers connected!");
    } else {
      RCLCPP_WARN(this->get_logger(), "Some Nav2 servers not available (may start later)");
    }
  }

  /**
   * @brief Callback for goal response
   */
  void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      current_goal_handle_ = goal_handle;
    }
  }

  /**
   * @brief Callback for navigation feedback
   */
  void feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {
    RCLCPP_DEBUG(this->get_logger(), 
      "Distance remaining: %.2f m", feedback->distance_remaining);
  }

  /**
   * @brief Callback for navigation result
   */
  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Navigation SUCCEEDED!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Navigation ABORTED!");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Navigation CANCELED!");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    current_goal_handle_ = nullptr;
  }

  /**
   * @brief Demo function to send a test goal (optional)
   */
  void send_demo_goal()
  {
    if (timer_) {
      timer_->cancel();
    }
    send_goal(1.0, 1.0, 0.0);
  }

  // Action Clients
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_client_;
  // Current Goal Handle
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_handle_;
  
  // Timer for demo
  rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief Main entry point
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigatorNode>());
  rclcpp::shutdown();
  return 0;
}