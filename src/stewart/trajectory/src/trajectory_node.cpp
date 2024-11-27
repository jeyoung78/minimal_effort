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

  TrajectoryNode::constructCircleTrajectory();
  TrajectoryNode::constructSquareTrajectory();
}

void TrajectoryNode::ballDetectionCallback(
    nav_msgs::msg::Odometry msg)
{
  double threshold = 0.01;

  double currPosX = msg.pose.pose.position.x;
  double currPosY = msg.pose.pose.position.y;
  double errorX = currentGoal.x - currPosX;
  double errorY = currentGoal.y - currPosY;

  double euclideanError = sqrt(pow(errorX, 2) + pow(errorY, 2));

  if (euclideanError < threshold)
  {
    newGoal = trajectory[0];
    ball_detection_sub_->publish(newGoal);

    trajectory.erase(trajectory.begin);
    trajectory.push_back(newGoal);
  }
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
  double offset = 10;
  double xMax = 250 - offset;
  double yMax = 250 - offset;
  double pointsPerEdge = numPoints / 4;

  double xInc = xMax / pointsPerEdge;
  double yInc = yMax / pointsPerEdge;

  for (int i = 0; i < pointsPerEdge; i++)
  {
    geometry_msgs::msg::Pose2D point;
    point.x = i * xInc;
    point.y = 0;
    trajectory.push_back(point);
  }
  for (int j = 0; j < pointsPerEdge; j++)
  {
    geometry_msgs::msg::Pose2D point;
    point.x = 0;
    point.y = j * yInc;
    trajectory.push_back(point);
  }
  for (int i = (int)pointsPerEdge; i > 0; i--)
  {
    geometry_msgs::msg::Pose2D point;
    point.x = i * xInc;
    point.y = 0;
    trajectory.push_back(point);
  }
  for (int j = (int)poitsPerEdge; j > 0; j--)
  {
    geometry_msgs::msg::Pose2D point;
    point.x = 0;
    point.y = j * yInc;
    trajectory.push_back(point);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
