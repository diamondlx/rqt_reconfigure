#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"

void chatterCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::cout << "I heard: [" << msg->data << "]" << std::endl;
  //printf("fff");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("listener_float");

  auto sub = node->create_subscription<std_msgs::msg::Float32>(
    "chatter", chatterCallback, rmw_qos_profile_default);

  rclcpp::spin(node);

  return 0;
}