#include <chrono>
#include <memory>
#include <sstream>

#include "camera_driver_node.hpp"

CameraDriverNode::CameraDriverNode() : Node("camera_driver") {
  // Initialize publisher
  image_publisher_ = image_transport::create_publisher(this, "camera/image");

  this->declare_parameter<int>("frames_per_second", 30);
  this->declare_parameter<bool>("enable_perspective_transform", false);
  frames_per_second_ = this->get_parameter("frames_per_second").as_int();
  enable_perspective_transform_ = this->get_parameter("enable_perspective_transform").as_bool();

  // Initialize OpenCV video capture
  cap_.open(0); // Open the default webcam (ID 0)
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the webcam.");
    rclcpp::shutdown();
  }

  // Start timer to capture and publish frames
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(frames_per_second_),  // ~30 FPS
    std::bind(&CameraDriverNode::publish_frame, this));
}

void CameraDriverNode::publish_frame() {
  cv::Mat frame;
  cap_ >> frame;  // Capture a frame

  if (frame.empty()) {
    RCLCPP_WARN(this->get_logger(), "Captured an empty frame.");
    return;
  }

  // Check if all pixel values are zero
  cv::Mat gray;
  if (frame.channels() > 1) {
      // Convert to grayscale if the image has multiple channels
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  } else {
      gray = frame;
  }
  if (cv::countNonZero(gray) == 0) {
    RCLCPP_WARN(this->get_logger(), "Completely black");
    return;
  }

  if (enable_perspective_transform_) {
    frame = do_perspective_transform(frame);
  }

  // Convert OpenCV image to ROS message
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  image_publisher_.publish(msg);
}

cv::Mat CameraDriverNode::do_perspective_transform(cv::Mat & frame) {
  RCLCPP_INFO(this->get_logger(), "A");
  // Preprocess the image to get a binary image
  cv::Mat binary = preprocessImage(frame);

  RCLCPP_INFO(this->get_logger(), "B");
  // Detect tag corners
  std::vector<cv::Point2f> tag_corners = detectTagCorners(binary);

  RCLCPP_INFO(this->get_logger(), "C");
  // Find the 4 outermost corners of the board
  std::vector<cv::Point2f> board_corners = findBoardCorners(tag_corners);

  RCLCPP_INFO(this->get_logger(), "D");
  // Perform the perspective transform
  cv::Mat output = warpBoard(frame, board_corners);
  
  return output;
}

cv::Mat CameraDriverNode::preprocessImage(const cv::Mat& frame) {
  cv::Mat gray, binary;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 2);
  return binary;
}

std::vector<cv::Point2f> CameraDriverNode::detectTagCorners(const cv::Mat& binary) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Point2f> tag_corners;

  // Find contours in the binary image
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto& contour : contours) {
    double area = cv::contourArea(contour);
    if (area < 100 || area > 5000) { // Filter by area (adjust thresholds as needed)
      continue;
    }

    // Approximate the contour to a polygon
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);

    // Check if the polygon is a quadrilateral
    if (approx.size() == 4 && cv::isContourConvex(approx)) {
      for (const auto& point : approx) {
        tag_corners.push_back(point);
      }
    }
  }

  return tag_corners;
}

std::vector<cv::Point2f> CameraDriverNode::findBoardCorners(const std::vector<cv::Point2f>& tag_corners) {
  // Sort points to find outermost ones
  RCLCPP_INFO(this->get_logger(), "11");
  cv::Point2f top_left = *std::min_element(tag_corners.begin(), tag_corners.end(),
                                            [](cv::Point2f a, cv::Point2f b) { return a.x + a.y < b.x + b.y; });
  RCLCPP_INFO(this->get_logger(), "22");
  cv::Point2f top_right = *std::max_element(tag_corners.begin(), tag_corners.end(),
                                            [](cv::Point2f a, cv::Point2f b) { return a.x - a.y < b.x - b.y; });
  RCLCPP_INFO(this->get_logger(), "33");
  cv::Point2f bottom_left = *std::max_element(tag_corners.begin(), tag_corners.end(),
                                              [](cv::Point2f a, cv::Point2f b) { return a.y - a.x < b.y - b.x; });
  RCLCPP_INFO(this->get_logger(), "44");
  cv::Point2f bottom_right = *std::max_element(tag_corners.begin(), tag_corners.end(),
                                                [](cv::Point2f a, cv::Point2f b) { return a.x + a.y > b.x + b.y; });

  return {top_left, top_right, bottom_right, bottom_left};
}

cv::Mat CameraDriverNode::warpBoard(const cv::Mat& frame, const std::vector<cv::Point2f>& board_corners) {
  const int grid_size = 6;
  const float square_size = 100.0f;

  // Define destination points for the perspective warp
  std::vector<cv::Point2f> destination_points = {
    {0, 0},
    {grid_size * square_size, 0},
    {grid_size * square_size, grid_size * square_size},
    {0, grid_size * square_size}
  };

  RCLCPP_INFO(this->get_logger(), "1");
  // Compute the homography matrix
  cv::Mat homography = cv::getPerspectiveTransform(board_corners, destination_points);

  RCLCPP_INFO(this->get_logger(), "2");
  // Warp the perspective
  cv::Mat output;
  cv::warpPerspective(frame, output, homography, cv::Size(grid_size * square_size, grid_size * square_size));
  return output;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraDriverNode>());
  rclcpp::shutdown();
  return 0;
}
