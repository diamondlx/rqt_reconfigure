#include <iostream>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("get_parameters");

  auto parameter_service = std::make_shared<rclcpp::parameter_service::ParameterService>(node);

  auto parameters_client = std::make_shared<rclcpp::parameter_client::SyncParametersClient>(node);
  //Set parameters
  auto set_parameters_results = parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("parameter_number", 1),
    rclcpp::parameter::ParameterVariant("parameter_string", "hello"),
    rclcpp::parameter::ParameterVariant("parameter_boolean", true),

  });
  for (auto & result : set_parameters_results) {
    if (!result.successful) {
      std::cerr << "Failed to set parameter: " << result.reason << std::endl;
    } else {
      std::cerr << "Parameter set" << std::endl;
    }
  }

  // Get parameters
  for (auto & parameter : parameters_client->get_parameters({"parameter_number", "parameter_string", "parameter_boolean"})) {
    std::cout << "Parameter name: " << parameter.get_name() << std::endl;
    std::cout << "Parameter value (" << parameter.get_type_name() << "): " <<
      parameter.value_to_string() << std::endl;
  }

  return 0;
}