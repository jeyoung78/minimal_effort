#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <cmath>

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

  static constexpr int ADVERTISING_FREQ = 30;

private:
  // Ball Perception
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ball_detection_sub_;
  void ballDetectionCallback(
      nav_msgs::msg::Odometry msg);

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr norm_vec_pub_;

  // Ball Trajectory
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr ball_goal_sub_;
  void ballGoalCallback(
      geometry_msgs::msg::Pose2D msg);

  geometry_msgs::msg::Pose2D currentGoal;

  // PID memory
  nav_msgs::msg::Odometry prevDetection;

  // PID errors
  double prevErrorX = 0;
  double prevErrorY = 0;

  // PID Integrals
  double integralX = 0;
  double integralY = 0;
  // std::vector<double> integral_vector_x;
  // std::vector<double> integral_vector_y;

  // PID Parameters
  double kp_;
  double ki_;
  double kd_;
  double k_;
};

#endif
