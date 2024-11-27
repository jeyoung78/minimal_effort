#ifndef PERCEPTION_NODE_HPP_
#define PERCEPTION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>

class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void checkTrajectory();
    void updateRegionTimes(double x, double y);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr traj_type_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr normal_publisher_;

    rclcpp::TimerBase::SharedPtr timer_; // New timer
    rclcpp::Clock::SharedPtr clock_;

    // Time trackers for region persistence
    rclcpp::Time last_in_origin_; // Last time ball was in the "origin" region
    rclcpp::Time last_in_square_; // Last time ball was in the "square" region
    rclcpp::Time last_in_circle_; // Last time ball was in the "circle" region
};

#endif // PERCEPTION_NODE_HPP_
