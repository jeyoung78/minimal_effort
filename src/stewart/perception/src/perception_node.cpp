#include "perception_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <cmath>

PerceptionNode::PerceptionNode() : Node("perception_node") {
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image", 10, 
        std::bind(&PerceptionNode::imageCallback, this, std::placeholders::_1));

    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/ball_odometry", 10);
    debug_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/debug_image", 10);

    RCLCPP_INFO(this->get_logger(), "PerceptionNode initialized");
}

void PerceptionNode::initializeKalmanFilter() {
    kalman_filter_ = cv::KalmanFilter(4, 2, 0); // 4 states (x, y, vx, vy), 2 measurements (x, y)

    // State transition matrix (A)
    kalman_filter_.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    // Measurement matrix (H)
    kalman_filter_.measurementMatrix = (cv::Mat_<float>(2, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0);

    // Process noise covariance matrix (Q)
    cv::setIdentity(kalman_filter_.processNoiseCov, cv::Scalar::all(1e-2));

    // Measurement noise covariance matrix (R)
    cv::setIdentity(kalman_filter_.measurementNoiseCov, cv::Scalar::all(1e-1));

    // Error covariance matrix (P)
    cv::setIdentity(kalman_filter_.errorCovPost, cv::Scalar::all(1));

    // Initial state
    state_ = cv::Mat::zeros(4, 1, CV_32F);
    measurement_ = cv::Mat::zeros(2, 1, CV_32F);

    is_kalman_initialized_ = true;
}


void PerceptionNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat frame;

    try {
        frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Convert to HSV for better color segmentation
    cv::Mat hsv_image;
    cv::cvtColor(frame, hsv_image, cv::COLOR_BGR2HSV);

    // Define the HSV range for an orange ping pong ball
    cv::Scalar lower_orange(0, 150, 80);  // Lower bound of orange
    cv::Scalar upper_orange(150, 255, 255); // Upper bound of orange

    // Threshold the HSV image to get only orange colors
    cv::Mat mask;
    cv::inRange(hsv_image, lower_orange, upper_orange, mask);

    // Find contours of the thresholded image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Publish the debug image (mask)
    try {
        // Draw contours on the original frame
        cv::Mat debug_frame = frame.clone(); // Clone the original frame for visualization
        cv::drawContours(debug_frame, contours, -1, cv::Scalar(0, 255, 0), 2); // Draw all contours in green with a thickness of 2
        
        auto debug_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_frame).toImageMsg();
        debug_msg->header.stamp = this->get_clock()->now();
        debug_image_publisher_->publish(*debug_msg);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception when publishing debug image: %s", e.what());
    }

    if (!contours.empty()) {
        // Find the largest contour (assuming it's the ball)
        auto max_contour = std::max_element(contours.begin(), contours.end(),
                                            [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
                                                return cv::contourArea(a) < cv::contourArea(b);
                                            });

        // Calculate the radius of the ball
        double area = cv::contourArea(*max_contour);
        double radius = std::sqrt(area / CV_PI); // Approximate radius from area

        // Define minimum and maximum radius thresholds
        const double MIN_RADIUS = 20.0; // Adjust this based on your application
        const double MAX_RADIUS = 50.0; // Adjust this based on your application

        // Only proceed if the radius is within the desired range
        if (radius >= MIN_RADIUS && radius <= MAX_RADIUS) {
            // Compute the center of the ball
            cv::Moments moments = cv::moments(*max_contour);
            double cx = moments.m10 / moments.m00;
            double cy = moments.m01 / moments.m00;

            if (!is_kalman_initialized_) {
                initializeKalmanFilter();
                state_.at<float>(0) = static_cast<float>(cx);
                state_.at<float>(1) = static_cast<float>(cy);
                kalman_filter_.statePost = state_;
            }

            // Predict the state
            cv::Mat prediction = kalman_filter_.predict();
            float velocity_x = prediction.at<float>(2);
            float velocity_y = prediction.at<float>(3);

            // Correct with measurement
            measurement_.at<float>(0) = static_cast<float>(cx);
            measurement_.at<float>(1) = static_cast<float>(cy);
            cv::Mat estimated = kalman_filter_.correct(measurement_);

            // Extract covariance
            const cv::Mat &P = kalman_filter_.errorCovPost;

            // Publish the odometry message
            auto odometry_msg = nav_msgs::msg::Odometry();
            odometry_msg.header.stamp = this->get_clock()->now();
            odometry_msg.header.frame_id = "camera_frame";
            odometry_msg.child_frame_id = "ball";

            // Fill pose
            // double x = estimated.at<float>(0) - 300;
            // double y = estimated.at<float>(1) - 220;
            double x = cx - 300;
            double y = cy - 220;
            double theta = -45 * M_PI / 180;

            odometry_msg.pose.pose.position.x = -x*cos(theta) + y*sin(theta); // Smoothed x
            odometry_msg.pose.pose.position.y = x*sin(theta) + y*cos(theta); // Smoothed y
            odometry_msg.pose.pose.position.z = 0.0;

            // Orientation (dummy value)
            odometry_msg.pose.pose.orientation.w = 1.0;

            odometry_publisher_->publish(odometry_msg);

            RCLCPP_INFO(this->get_logger(),
                        "Predicted ball position: x=%.2f, y=%.2f | Velocity: vx=%.2f, vy=%.2f",
                        odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y,
                        velocity_x, velocity_y);
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "Contour rejected: radius=%.2f (not within [%.2f, %.2f])", radius, MIN_RADIUS, MAX_RADIUS);

            auto odometry_msg = nav_msgs::msg::Odometry();
            odometry_msg.pose.pose.orientation.w = -1.0;
            odometry_publisher_->publish(odometry_msg);
        }
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
