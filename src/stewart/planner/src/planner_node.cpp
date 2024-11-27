#include <chrono>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner")
{
  // Setup publisher for normal vector publishing
  norm_vec_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/normal_vector", ADVERTISING_FREQ);

  // Subscribing to ball detection msg
  ball_detection_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ball_odometry", ADVERTISING_FREQ,
      std::bind(
          &PlannerNode::ballDetectionCallback, this,
          std::placeholders::_1));

  // Subscribing to ball goal msg
  ball_goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      "/ball_goal", ADVERTISING_FREQ,
      std::bind(
          &PlannerNode::ballGoalCallback, this,
          std::placeholders::_1));
}

void PlannerNode::ballGoalCallback(
    geometry_msgs::msg::Pose2D msg)
{
  currentGoal = msg;
}

void PlannerNode::ballDetectionCallback(
    nav_msgs::msg::Odometry msg)
{
  // Get PID parameters from param file
  kp = this->get_parameter("kp").as_double();
  ki = this->get_parameter("ki").as_double();
  kd = this->get_parameter("kd").as_double();
  k = this->get_parameter("k").as_double();

  // Calculate error
  double currPosX = msg.pose.pose.position.x;
  double currPosY = msg.pose.pose.position.y;
  double errorX = currentGoal.x - currPosX;
  double errorY = currentGoal.y - currPosY;

  double dt = msg.header.stamp.nanosec - prevDetection.header.stamp.nanosec;

  // Calcualte Integrals
  integralX = integralX + errorX * dt;
  integralY = integralY + errorY * dt;

  // Calculate Derivatives
  double derivativeX = (errorX - prevErrorX) / dt;
  double derivativeY = (errorY - prevErrorY) / dt;

  // Calculate PID control output
  double outputX = (kp * errorX) + (ki * integralX) + (kd * derivativeX);
  double outputY = (kp * errorY) + (ki * integralY) + (kd * derivativeY);

  // Calculate frame angles
  double theta = atan2(outputY, outputX);
  double phi = k * sqrt(pow(outputX, 2) + pow(outputY, 2));

  // Calculate normal vectors for plane
  double z = cos(phi);
  double r = sin(phi);
  double x = r * cos(theta);
  double y = r * sin(theta);

  geometry_msgs::msg::Vector3 normVector;
  normVector.x = x;
  normVector.y = y;
  normVector.z = z;

  norm_vec_pub_->publish(normVector);

  // Update stored variables
  prevErrorX = errorX;
  prevErrorY = errorY;
  prevDetection = msg;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
