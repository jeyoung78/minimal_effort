#include <chrono>
#include <memory>

#include "perception_node.hpp"

PerceptionNode::PerceptionNode() : Node("perception") {

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
