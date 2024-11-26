#ifndef PERCEPTION_NODE_HPP_
#define PERCEPTION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>

class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void initializeKalmanFilter();

    cv::KalmanFilter kalman_filter_;
    cv::Mat state_;  // [x, y, vx, vy]
    cv::Mat measurement_; // [x, y]

    bool is_kalman_initialized_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
};

#endif // PERCEPTION_NODE_HPP_
