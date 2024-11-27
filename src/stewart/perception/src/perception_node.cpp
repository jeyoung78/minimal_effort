#include "perception_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <cmath>

PerceptionNode::PerceptionNode() : Node("perception_node") {
    clock_ = this->get_clock();

    // Initialize time variables
    auto zero_time = rclcpp::Time(0, 0, clock_->get_clock_type()); // Initialize with same clock type
    last_in_origin_ = zero_time;
    last_in_square_ = zero_time;
    last_in_circle_ = zero_time;

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image", 10, 
        std::bind(&PerceptionNode::imageCallback, this, std::placeholders::_1));

    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/ball_odometry", 10);
    debug_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/debug_image", 10);
    traj_type_publisher_ = this->create_publisher<std_msgs::msg::String>("/traj_type", 10);
    normal_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/normal_vector", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PerceptionNode::checkTrajectory, this));

    RCLCPP_INFO(this->get_logger(), "PerceptionNode initialized");
}

void PerceptionNode::checkTrajectory() {
    const double REQUIRED_DURATION = 3.0;

    auto now = clock_->now(); // Use consistent clock source

    if (last_in_origin_ != rclcpp::Time(0, 0, clock_->get_clock_type()) &&
        (now - last_in_origin_).seconds() >= REQUIRED_DURATION) {
        auto msg = std_msgs::msg::String();
        msg.data = "origin";
        traj_type_publisher_->publish(msg);
        geometry_msgs::msg::Vector3 n;
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        RCLCPP_INFO(this->get_logger(), "Published trajectory type: origin");
        last_in_origin_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    }

    if (last_in_square_ != rclcpp::Time(0, 0, clock_->get_clock_type()) &&
        (now - last_in_square_).seconds() >= REQUIRED_DURATION) {
        auto msg = std_msgs::msg::String();
        msg.data = "square";
        traj_type_publisher_->publish(msg);
        geometry_msgs::msg::Vector3 n;
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        RCLCPP_INFO(this->get_logger(), "Published trajectory type: square");
        last_in_square_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    }

    if (last_in_circle_ != rclcpp::Time(0, 0, clock_->get_clock_type()) &&
        (now - last_in_circle_).seconds() >= REQUIRED_DURATION) {
        auto msg = std_msgs::msg::String();
        msg.data = "circle";
        traj_type_publisher_->publish(msg);
        geometry_msgs::msg::Vector3 n;
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = 0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = -0.1132;
        n.y = 0.0;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        n.x = 0.0;
        n.y = -0.1132;
        n.z = 0.9936;
        normal_publisher_->publish(n);
        RCLCPP_INFO(this->get_logger(), "Published trajectory type: circle");
        last_in_circle_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    }
}


void PerceptionNode::updateRegionTimes(double x, double y) {
    const double ORIGIN_X = 190, ORIGIN_Y = 160, ORIGIN_TOLERANCE = 50;
    const double SQUARE_X = -170, SQUARE_Y = 140, SQUARE_TOLERANCE = 50;
    const double CIRCLE_X = 130, CIRCLE_Y = -150, CIRCLE_TOLERANCE = 50;

    auto now = clock_->now(); // Use consistent clock source

    if (std::abs(x - ORIGIN_X) <= ORIGIN_TOLERANCE && std::abs(y - ORIGIN_Y) <= ORIGIN_TOLERANCE) {
        if (last_in_origin_ == rclcpp::Time(0, 0, clock_->get_clock_type())) {
            last_in_origin_ = now; // Start the timer
        }
    } else {
        last_in_origin_ = rclcpp::Time(0, 0, clock_->get_clock_type()); // Reset timer if the ball leaves the region
    }

    if (std::abs(x - SQUARE_X) <= SQUARE_TOLERANCE && std::abs(y - SQUARE_Y) <= SQUARE_TOLERANCE) {
        if (last_in_square_ == rclcpp::Time(0, 0, clock_->get_clock_type())) {
            last_in_square_ = now; // Start the timer
        }
    } else {
        last_in_square_ = rclcpp::Time(0, 0, clock_->get_clock_type()); // Reset timer if the ball leaves the region
    }

    if (std::abs(x - CIRCLE_X) <= CIRCLE_TOLERANCE && std::abs(y - CIRCLE_Y) <= CIRCLE_TOLERANCE) {
        if (last_in_circle_ == rclcpp::Time(0, 0, clock_->get_clock_type())) {
            last_in_circle_ = now; // Start the timer
        }
    } else {
        last_in_circle_ = rclcpp::Time(0, 0, clock_->get_clock_type()); // Reset timer if the ball leaves the region
    }
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
        debug_msg->header.stamp = clock_->now();
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

            // Publish the odometry message
            auto odometry_msg = nav_msgs::msg::Odometry();
            odometry_msg.header.stamp = clock_->now();
            odometry_msg.header.frame_id = "camera_frame";
            odometry_msg.child_frame_id = "ball";

            // Fill pose
            // double x = estimated.at<float>(0) - 300;
            // double y = estimated.at<float>(1) - 220;
            double x = cx - 300;
            double y = cy - 220;
            double theta = 135 * M_PI / 180;

            double temp = x;
            x = x * cos(theta) - y * sin(theta);
            y = temp * sin(theta) + y * cos(theta);

            odometry_msg.pose.pose.position.x = x; // Smoothed x
            odometry_msg.pose.pose.position.y = -y; // Smoothed y
            odometry_msg.pose.pose.position.z = 0.0;

            updateRegionTimes(x, -y);

            // Orientation (dummy value)
            odometry_msg.pose.pose.orientation.w = 1.0;

            odometry_publisher_->publish(odometry_msg);

            RCLCPP_INFO(this->get_logger(),
                        "Predicted ball position: x=%.2f, y=%.2f",
                        odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y);

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
