#ifndef TRAJECTORY_NODE_HPP_
#define TRAJECTORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

class TrajectoryNode : public rclcpp::Node
{
public:
  TrajectoryNode();

  static constexpr int ADVERTISING_FREQ = 30;

private:
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr goal_point_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ball_detection_sub_;
  void ballDetectionCallback(nav_msgs::msg::Odometry msg);
  geometry_msgs::msg::Pose2D currentGoal;

  int numPoints;
  std::vector<geometry_msgs::msg::Pose2D> trajectory;
  int thesholdCounter;

  nav_msgs::msg::Odometry prevDetection;

  double prevErrorX = 0;
  double prevErrorY = 0;
  double vel_threshold_;

  void constructSquareTrajectory();
  void constructCircleTrajectory();
  void gotoOrigin();
};

#endif
