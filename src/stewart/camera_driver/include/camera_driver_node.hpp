#ifndef CAMERA_DRIVER_NODE_HPP_
#define CAMERA_DRIVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/core.hpp>

class CameraDriverNode : public rclcpp::Node {
  public:
    CameraDriverNode();
  
  private:
    void publishFrame();

    cv::Mat getHomography(cv::Mat& frame);
    cv::Mat preprocessImage(const cv::Mat& frame);
    std::vector<cv::Point2f> detectCheckerVertices(const cv::Mat& binary);
    std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> corrospondToIdealVertices(const std::vector<cv::Point2f>& checker_vertices);

    cv::flann::Index buildKDTree(const std::vector<cv::Point2f>& checker_vertices);
    
    int frames_per_second_;
    bool compute_homography_;
    bool debug_;

    int checkerboard_nx_;
    int checkerboard_ny_;
    double checkerboard_segment_length_;

    cv::flann::Index kdtree;

    image_transport::Publisher image_publisher_;
    image_transport::Publisher image_debug_publisher_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif 