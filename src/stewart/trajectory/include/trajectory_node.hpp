#ifndef TRAJECTORY_NODE_HPP_
#define TRAJECTORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
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

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr traj_type_sub_;
  void trajTypeCallback(std_msgs::msg::String msg);
  std::string curr_traj_type;

  int numPoints;
  std::vector<geometry_msgs::msg::Pose2D> trajectory;
  int thesholdCounter;
  int time_between_goals_;
  double timeSinceLastGoal;

  int time_count;
  double errorThreshold;
  int timeThreshold;

  int timeThresholdOrigin;
  int timeThresholdSquare;
  int timeThresholdCircle;

  double last_goal_sent_time;

  nav_msgs::msg::Odometry prevDetection;

  void constructSquareTrajectory();
  void constructCircleTrajectory();
  void gotoOrigin();
};

#endif
