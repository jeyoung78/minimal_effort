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

  this->declare_parameter<double>("kp", false);
  this->declare_parameter<double>("ki", false);
  this->declare_parameter<double>("kd", false);
  this->declare_parameter<double>("k", false);
  kp_ = this->get_parameter("kp").as_double();
  ki_ = this->get_parameter("ki").as_double();
  kd_ = this->get_parameter("kd").as_double();
  k_ = this->get_parameter("k").as_double();

}

void PlannerNode::ballGoalCallback(
    geometry_msgs::msg::Pose2D msg)
{
  currentGoal = msg;
}

void PlannerNode::ballDetectionCallback(
    nav_msgs::msg::Odometry msg)
{
  if (msg.pose.pose.orientation.w < 0.0) {
    integralX = 0;
    integralY = 0;

    RCLCPP_INFO(this->get_logger(), "integralX asd %f", integralX);
    RCLCPP_INFO(this->get_logger(), "integralY asd %f", integralY);

    geometry_msgs::msg::Vector3 normVector;
    normVector.x = 0.0;
    normVector.y = 0.0;
    normVector.z = 1.0;

    norm_vec_pub_->publish(normVector);

    prevDetection = msg;
    return;
  }

  if (prevDetection.header.stamp.nanosec == 0 || prevDetection.header.stamp.sec == 0) { 
    prevDetection = msg;
    return; 
  }

  // Calculate error
  double currPosX = msg.pose.pose.position.x;
  double currPosY = msg.pose.pose.position.y;
  double errorX = currPosX - currentGoal.x;
  double errorY = currPosY - currentGoal.y;

  RCLCPP_INFO(this->get_logger(), "output from errorX %f", kp_ * errorX);
  RCLCPP_INFO(this->get_logger(), "otuput from errorY %f", kp_ * errorY);

  double t_now = msg.header.stamp.sec * pow(10, 9) + msg.header.stamp.nanosec;
  double t_prev = prevDetection.header.stamp.sec * pow(10, 9) + prevDetection.header.stamp.nanosec;
  double dt = (t_now - t_prev) / pow(10, 9);

  RCLCPP_INFO(this->get_logger(), "msg.header.stamp.sec %d", msg.header.stamp.sec);
  RCLCPP_INFO(this->get_logger(), "prevDetection.header.stamp.sec %d", prevDetection.header.stamp.sec);
  RCLCPP_INFO(this->get_logger(), "dt %f", dt);

  // Calcualte Integrals
  integralX = integralX + errorX * dt;
  integralY = integralY + errorY * dt;

  RCLCPP_INFO(this->get_logger(), "output from integralX %f", ki_ * integralX);
  RCLCPP_INFO(this->get_logger(), "output from integralY %f", ki_ * integralY);

  // Calculate Derivatives
  double derivativeX = (errorX - prevErrorX) / dt;
  double derivativeY = (errorY - prevErrorY) / dt;

  RCLCPP_INFO(this->get_logger(), "output from derivativeX %f", kd_ * derivativeX);
  RCLCPP_INFO(this->get_logger(), "output from derivativeY %f", kd_ * derivativeY);


  // Calculate PID control output
  double outputX = (kp_ * errorX) + (ki_ * integralX) + (kd_ * derivativeX);
  double outputY = (kp_ * errorY) + (ki_ * integralY) + (kd_ * derivativeY);

  // Calculate frame angles
  double theta = atan2(outputY, outputX);
  double phi = k_ * sqrt(pow(outputX, 2) + pow(outputY, 2)) * M_PI / 180;

  // Calculate normal vectors for plane
  double z = cos(phi);
  double r = sin(phi);
  double x = r * cos(theta);
  double y = r * sin(theta);

  RCLCPP_INFO(this->get_logger(), "command N theta=%f phi=%f z=%f r=%f x=%f y=%f", theta, phi, z, r, x, y);

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
