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

  // Setup publisher for normal vector publishing
  goal_point_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/ball_goal", ADVERTISING_FREQ);

  this->declare_parameter<double>("vel_threshold", 0.0);
  vel_threshold_ = this->get_parameter("vel_threshold").as_double();

  // TrajectoryNode::constructCircleTrajectory();
  TrajectoryNode::constructSquareTrajectory();
  // TrajectoryNode::gotoOrigin();
}

void TrajectoryNode::gotoOrigin()
{
  geometry_msgs::msg::Pose2D origin;
  origin.x = 0;
  origin.y = 0;
  goal_point_pub_->publish(origin);
}

void TrajectoryNode::ballDetectionCallback(
    nav_msgs::msg::Odometry msg)
{
  if (msg.pose.pose.orientation.w < 0.0) {
    prevDetection = msg;
    return;
  }

  if (prevDetection.header.stamp.nanosec == 0 || prevDetection.header.stamp.sec == 0) { 
    prevDetection = msg;
    return; 
  }

  double currPosX = msg.pose.pose.position.x;
  double currPosY = msg.pose.pose.position.y;
  double errorX = currentGoal.x - currPosX;
  double errorY = currentGoal.y - currPosY;

  double t_now = msg.header.stamp.sec * pow(10, 9) + msg.header.stamp.nanosec;
  double t_prev = prevDetection.header.stamp.sec * pow(10, 9) + prevDetection.header.stamp.nanosec;
  double dt = (t_now - t_prev) / pow(10, 9);

  // Calculate Derivatives
  double derivativeX = (errorX - prevErrorX) / dt;
  double derivativeY = (errorY - prevErrorY) / dt;

  double total_vel = sqrt(pow(derivativeX, 2)+ pow(derivativeY, 2));

  if (total_vel < vel_threshold_)
  {
    thesholdCounter++;
    geometry_msgs::msg::Pose2D newGoal = trajectory[0];
    currentGoal = newGoal;

    RCLCPP_INFO(this->get_logger(), "Publishing new goal %f %f", newGoal.x, newGoal.y);
    goal_point_pub_->publish(newGoal);

    trajectory.erase(trajectory.begin());
    trajectory.push_back(newGoal);
  }

  prevDetection = msg;
}

void TrajectoryNode::constructCircleTrajectory()
{
  double offset = 10;
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
