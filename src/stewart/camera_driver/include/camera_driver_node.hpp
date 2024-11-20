#ifndef CAMERA_DRIVER_NODE_HPP_
#define CAMERA_DRIVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class CameraDriverNode : public rclcpp::Node {
  public:
    CameraDriverNode();
  
  private:
    void publish_frame();

    cv::Mat do_perspective_transform(cv::Mat& frame);
    cv::Mat preprocessImage(const cv::Mat& frame);
    std::vector<cv::Point2f> detectTagCorners(const cv::Mat& binary);
    std::vector<cv::Point2f> findBoardCorners(const std::vector<cv::Point2f>& tag_corners);
    cv::Mat warpBoard(const cv::Mat& frame, const std::vector<cv::Point2f>& board_corners);

    int frames_per_second_;
    bool enable_perspective_transform_;

    image_transport::Publisher image_publisher_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif 
