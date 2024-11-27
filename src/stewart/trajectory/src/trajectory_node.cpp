#include <chrono>
#include <memory>

#include "trajectory_node.hpp"

TrajectoryNode::TrajectoryNode() : Node("trajectory")
{
  // Subscribing to ball detection msg
  ball_detection_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ball_odometry", ADVERTISING_FREQ,
      std::bind(
          &TrajectoryNode::ballDetectionCallback, this,
          std::placeholders::_1));

  // Subscribing to traj_type_sub_ msg
  traj_type_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/traj_type", ADVERTISING_FREQ,
      std::bind(
          &TrajectoryNode::trajTypeCallback, this,
          std::placeholders::_1));

  // Setup publisher for normal vector publishing
  goal_point_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/ball_goal", ADVERTISING_FREQ);

  this->declare_parameter<double>("time_between_goals", 2.5);
  time_between_goals_ = this->get_parameter("time_between_goals").as_double();

  // TrajectoryNode::constructCircleTrajectory();
  // TrajectoryNode::constructSquareTrajectory();
  TrajectoryNode::gotoOrigin();

  this->declare_parameter<double>("error_threshold", 0.0);
  errorThreshold = this->get_parameter("error_threshold").as_double();

  this->declare_parameter<int>("time_threshold_origin", 0);
  timeThresholdOrigin = this->get_parameter("time_threshold_origin").as_int();
  this->declare_parameter<int>("time_threshold_circle", 0);
  timeThresholdCircle = this->get_parameter("time_threshold_circle").as_int();
  this->declare_parameter<int>("time_threshold_square", 0);
  timeThresholdSquare = this->get_parameter("time_threshold_square").as_int();

  timeThreshold = timeThresholdOrigin;
}

void TrajectoryNode::trajTypeCallback(std_msgs::msg::String msg)
{
  trajectory.clear();
  if (msg.data == "circle")
  {
    RCLCPP_INFO(this->get_logger(), "changed to circle");
    TrajectoryNode::constructCircleTrajectory();
    timeThreshold = timeThresholdCircle;
  }
  else if (msg.data == "square")
  {
    RCLCPP_INFO(this->get_logger(), "changed to square");
    curr_traj_type = "square";
    TrajectoryNode::constructSquareTrajectory();
    timeThreshold = timeThresholdSquare;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "changed to origin");
    curr_traj_type = "origin";
    TrajectoryNode::gotoOrigin();
    timeThreshold = timeThresholdOrigin;
  }
}

void TrajectoryNode::gotoOrigin()
{
  geometry_msgs::msg::Pose2D origin;
  origin.x = 0;
  origin.y = 0;
  goal_point_pub_->publish(origin);

  trajectory.push_back(origin);
}

void TrajectoryNode::ballDetectionCallback(
    nav_msgs::msg::Odometry msg)
{
  if (msg.pose.pose.orientation.w < 0.0) {
    return;
  }

  double currPosX = msg.pose.pose.position.x;
  double currPosY = msg.pose.pose.position.y;
  double errorX = currPosX - currentGoal.x;
  double errorY = currPosY - currentGoal.y;
  double total_error = sqrt(pow(errorX, 2) + pow(errorY, 2));

  RCLCPP_INFO(this->get_logger(), "total error %f", total_error);

  // count up time at place
  if (total_error < errorThreshold)
    time_count++;
  else
    time_count = 0;

  RCLCPP_INFO(this->get_logger(), "time count %d", time_count);
  RCLCPP_INFO(this->get_logger(), "time timeThreshold %d", timeThreshold);

  // If at point long enough, then
  if (time_count > timeThreshold)
  {
    time_count = 0;
    geometry_msgs::msg::Pose2D newGoal = trajectory[0];
    currentGoal = newGoal;

    RCLCPP_INFO(this->get_logger(), "Publishing new goal %f %f", newGoal.x, newGoal.y);
    goal_point_pub_->publish(newGoal);

    if (trajectory.size() == 1) {
      return;
    }

    trajectory.erase(trajectory.begin());
    trajectory.push_back(newGoal);
  }
}

void TrajectoryNode::constructCircleTrajectory()
{
  double offset = 150;
  double xMax = 250 - offset;
  double yMax = 250 - offset;
  double thetaInc = (2 * M_PI) / numPoints;

  for (int i = 0; i < numPoints; i++)
  {
    geometry_msgs::msg::Pose2D point;
    point.x = xMax * cos(thetaInc * i);
    point.y = yMax * sin(thetaInc * i);

    trajectory.push_back(point);
  }
}

void TrajectoryNode::constructSquareTrajectory()
{
  double offset = 100;
  double xMax = 250 - offset;
  double yMax = 250 - offset;

  geometry_msgs::msg::Pose2D point;
  point.x = 0;
  point.y = -yMax;
  trajectory.push_back(point);

  point.x = -xMax;
  point.y = 0;
  trajectory.push_back(point);

  point.x = 0;
  point.y = yMax;
  trajectory.push_back(point);

  point.x = xMax;
  point.y = 0;
  trajectory.push_back(point);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
