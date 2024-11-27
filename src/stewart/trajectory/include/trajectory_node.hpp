#ifndef TRAJECTORY_NODE_HPP_
#define TRAJECTORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <cmath>

class TrajectoryNode : public rclcpp::Node
{
public:
  PlannerNode();

  static constexpr int ADVERTISING_FREQ = 30;

private:
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr goal_point_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ball_detection_sub_;
  void ballDetectionCallback(
      nav_msgs::msg::Odometry msg);
  geometry_msgs::msg::Pose2D currentGoal;

  int numPoints;
  std::vector<geometry_msgs::msg::Pose2D> trajectory;

  void constructSquareTrajectory();
  void constructCircleTrajectory();
};

#endif
