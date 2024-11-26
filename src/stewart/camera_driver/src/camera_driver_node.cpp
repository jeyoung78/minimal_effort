#include <chrono>
#include <memory>
#include <sstream>
#include <unordered_set>

#include "camera_driver_node.hpp"

// Define a hash function for cv::Point2f
struct Point2fHash {
    size_t operator()(const cv::Point2f& point) const {
        return std::hash<float>()(point.x) ^ (std::hash<float>()(point.y) << 1);
    }
};

// Define an equality operator for cv::Point2f
struct Point2fEqual {
    bool operator()(const cv::Point2f& a, const cv::Point2f& b) const {
        return std::abs(a.x - b.x) < 1e-5 && std::abs(a.y - b.y) < 1e-5;
    }
};

CameraDriverNode::CameraDriverNode() : Node("camera_driver") {
  // Initialize publisher
  image_publisher_ = image_transport::create_publisher(this, "camera/image");

  this->declare_parameter<int>("frames_per_second", 30);
  this->declare_parameter<bool>("compute_homography", false);
  this->declare_parameter<bool>("debug", false);
  this->declare_parameter<bool>("checkerboard_nx", false);
  this->declare_parameter<bool>("checkerboard_ny", false);
  this->declare_parameter<bool>("checkerboard_segment_length", false);
  frames_per_second_ = this->get_parameter("frames_per_second").as_int();
  compute_homography_ = this->get_parameter("compute_homography").as_bool();
  debug_ = this->get_parameter("debug").as_bool();
  checkerboard_nx_ = this->get_parameter("checkerboard_nx").as_int();
  checkerboard_ny_ = this->get_parameter("checkerboard_ny").as_int();
  checkerboard_segment_length_ = this->get_parameter("checkerboard_segment_length").as_double();

  if (debug_) {
    image_debug_publisher_ = image_transport::create_publisher(this, "camera/image_debug");
  }

  // Initialize OpenCV video capture
  cap_.open(0); // Open the default webcam (ID 0)
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the webcam.");
    rclcpp::shutdown();
  }

  // Start timer to capture and publish frames
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(frames_per_second_),  // ~30 FPS
    std::bind(&CameraDriverNode::publishFrame, this));
}

void CameraDriverNode::publishFrame() {
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

  if (compute_homography_) {
    frame = getHomography(frame);
  }

  // Convert OpenCV image to ROS message
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  image_publisher_.publish(msg);
}

cv::Mat CameraDriverNode::getHomography(cv::Mat & frame) {
  // Preprocess the image to get a binary image
  cv::Mat binary = preprocessImage(frame);
  // Detect tag corners
  std::vector<cv::Point2f> checker_vertices = detectCheckerVertices(binary);
  // Corrospond as many points to an ideal checkerboard pattern as possible using BFS and K-D Tree
  std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> ideal_checker_vertex_pairs = corrospondToIdealVertices(checker_vertices);
  // Calculate the homography matrix
  cv::Mat homography = cv::getPerspectiveTransform(ideal_checker_vertex_pairs.first, ideal_checker_vertex_pairs.second);
  
  return homography;
}

cv::Mat CameraDriverNode::preprocessImage(const cv::Mat& frame) {
  cv::Mat gray, binary;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 201, 30);
  return binary;
}

std::vector<cv::Point2f> CameraDriverNode::detectCheckerVertices(const cv::Mat& binary) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Point2f> checker_vertices;

  // Find contours in the binary image
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto& contour : contours) {
    double area = cv::contourArea(contour);
    if (area < 1000 || area > 5000) { // Filter by area (adjust thresholds as needed)
      continue;
    }

    // Approximate the contour to a polygon
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);

    // Check if the polygon is a quadrilateral
    if (approx.size() == 4 && cv::isContourConvex(approx)) {
      for (const auto& point : approx) {
        checker_vertices.push_back(point);
      }
    }
  }

  if (debug_) {
    // Convert binary image to a 3-channel BGR image for color drawing
    cv::Mat binary_with_contours;
    cv::cvtColor(binary, binary_with_contours, cv::COLOR_GRAY2BGR);

    // Draw contours in red
    cv::drawContours(binary_with_contours, contours, -1, cv::Scalar(0, 0, 255), 2);

    for (const auto & point : checker_vertices) {
      // Draw a circle at each corner point
      cv::circle(binary_with_contours, point, 5, cv::Scalar(0, 255, 0), -1); // Green color
    }
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", binary_with_contours).toImageMsg();
    image_debug_publisher_.publish(msg);
  }
  
  return checker_vertices;
}

std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> CameraDriverNode::corrospondToIdealVertices(const std::vector<cv::Point2f>& checker_vertices) {
  std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> checker_vertex_pairs;
  std::vector<cv::Point2f> ideal_checker_vertices;
  std::vector<cv::Point2f> nonideal_checker_vertices;

  // Step 1: Build k-d Tree
  cv::flann::Index kdtree = buildKDTree(checker_vertices);

  // Step 2: Find the four corners
  cv::Point2f top_left = *std::min_element(checker_vertices.begin(), checker_vertices.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
    return a.x + a.y < b.x + b.y;
  });

  cv::Point2f bottom_right = *std::max_element(checker_vertices.begin(), checker_vertices.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
    return a.x + a.y < b.x + b.y;
  });

  cv::Point2f top_right = *std::max_element(checker_vertices.begin(), checker_vertices.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
    return a.x - a.y > b.x - b.y;
  });

  cv::Point2f bottom_left = *std::min_element(checker_vertices.begin(), checker_vertices.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
    return a.x - a.y > b.x - b.y;
  });

  std::vector<cv::Point2f> corners = {
    top_left, 
    bottom_right, 
    top_right,
    bottom_left
  };

  for (const auto& corner : corners) {
    bool valid_corner = true;

    // Step 3: Select the top-left corner and find the two closest points
    std::vector<int> indices(2);
    std::vector<float> dists(2);
    cv::Mat query(1, 2, CV_32F);
    query.at<float>(0, 0) = corner.x;
    query.at<float>(0, 1) = corner.y;

    kdtree.knnSearch(query, indices, dists, 2); // Find 2 closest points

    // Compute traversal directions and starting point
    cv::Point2f dir_row = checker_vertices[indices[0]] - corner;
    cv::Point2f dir_col = checker_vertices[indices[1]] - corner;
    cv::Point2f starting_point;

    if (corner == top_left) {
      starting_point = cv::Point2f(0, 0);
    }
    else if (corner == bottom_right) {
      starting_point = cv::Point2f(checkerboard_nx_ * checkerboard_segment_length_, checkerboard_nx_ * checkerboard_segment_length_);
    }
    else if (corner == top_right) {
      starting_point = cv::Point2f(checkerboard_nx_ * checkerboard_segment_length_, 0);
    }
    else if (corner == bottom_left) {
      starting_point = cv::Point2f(0, checkerboard_nx_ * checkerboard_segment_length_);
    }

    // Step 4: BFS to traverse the grid
    std::queue<std::tuple<cv::Point2f, cv::Point2f, int, int>> q; // Store current point and its ideal location
    q.push({top_left, starting_point, 0, 0});

    std::unordered_set<cv::Point2f, Point2fHash, Point2fEqual> visited;

    while (!q.empty()) {
      auto [current_point, ideal_point, grid_point_x, grid_point_y] = q.front();
      q.pop();

      if (visited.count(current_point)) continue;
      visited.insert(current_point);

      nonideal_checker_vertices.push_back(current_point);
      ideal_checker_vertices.push_back(ideal_point);

      // Traverse to neighbors
      for (const auto& direction : {dir_row, dir_col, -dir_row, -dir_col}) {
        cv::Point2f next_point = current_point + direction;

        // Find the closest point in the detected vertices
        query.at<float>(0, 0) = next_point.x;
        query.at<float>(0, 1) = next_point.y;

        kdtree.knnSearch(query, indices, dists, 1); // Find the closest point
        cv::Point2f closest_point = checker_vertices[indices[0]];

        if (cv::norm(closest_point - next_point) < cv::norm(direction) * 0.1f) { // Threshold for matching
          cv::Point2f new_ideal_point = ideal_point;
          if (direction == dir_row) {
            ideal_point += cv::Point2f(checkerboard_segment_length_, 0);
            grid_point_x++;
          }
          else if (direction == dir_col) {
            ideal_point += cv::Point2f(0, checkerboard_segment_length_);
            grid_point_y++;
          }
          else if (direction == -dir_row) {
            ideal_point += cv::Point2f(-checkerboard_segment_length_, 0);
            grid_point_x--;
            if (grid_point_x < 0) { valid_corner = false; }
          }
          else if (direction == -dir_col) {
            ideal_point += cv::Point2f(0, -checkerboard_segment_length_);
            grid_point_y--;
            if (grid_point_y < 0) { valid_corner = false; }
          }
          q.push({closest_point, new_ideal_point, grid_point_x, grid_point_y});
        }
      }
    }

    if (valid_corner) {
      break;
    } else {
      nonideal_checker_vertices.clear();
      ideal_checker_vertices.clear();
    }
  }

  checker_vertex_pairs.first = nonideal_checker_vertices;
  checker_vertex_pairs.second = ideal_checker_vertices;

  return checker_vertex_pairs;
}

cv::flann::Index CameraDriverNode::buildKDTree(const std::vector<cv::Point2f>& checker_vertices) {
  cv::Mat data(checker_vertices.size(), 2, CV_32F);
  for (size_t i = 0; i < checker_vertices.size(); ++i) {
    data.at<float>(i, 0) = checker_vertices[i].x;
    data.at<float>(i, 1) = checker_vertices[i].y;
  }

  return cv::flann::Index(data, cv::flann::KDTreeIndexParams(1));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraDriverNode>());
  rclcpp::shutdown();
  return 0;
}