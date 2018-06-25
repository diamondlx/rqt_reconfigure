#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "cfg/testRclConfig.h"
#include "Server.hpp"
#include <vector>


void callback(std::vector<rclcpp::parameter::ParameterVariant> all_value) {
  printf("user callback\n");
  for (auto parameter : all_value) {
  	std::cout << parameter.get_name() << " ";
  	switch(parameter.get_type()) {
      case rclcpp::parameter::ParameterType::PARAMETER_BOOL:
        if(parameter.as_bool()) {
          printf("True\n");
        } else {
          printf("False\n");
        }
        continue;
      case rclcpp::parameter::ParameterType::PARAMETER_INTEGER:
        printf("%d\n", parameter.as_int());
        continue;
      case rclcpp::parameter::ParameterType::PARAMETER_DOUBLE:
        printf("%lf\n", parameter.as_double());
        continue;
      case rclcpp::parameter::ParameterType::PARAMETER_STRING:
        std::cout << parameter.as_string() << std::endl;
        continue;
    }
  }
}


int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("get_parameters3");

  auto ser_ptr = std::make_shared<rqt_reconfigure::Server<ConfigureVec>>(node, callback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}