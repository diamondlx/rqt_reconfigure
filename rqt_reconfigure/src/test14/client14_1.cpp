 #include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

example_interfaces::srv::AddTwoInts_Response::SharedPtr send_request(
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client,
  example_interfaces::srv::AddTwoInts_Request::SharedPtr request)
{
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return result.get();
  } else {
    return NULL;
  }
}

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_ints_client14_1");

  auto topic = std::string("add_two_ints_test");
  auto client = node->create_client<example_interfaces::srv::AddTwoInts>(topic);

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 2;
  request->b = 1;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      printf( "Interrupted while waiting for the service. Exiting.\n");
      return 0;
    }
    printf( "service not available, waiting again...\n");
  }
  printf("service is ready\n");
  // TODO(wjwwood): make it like `client->send_request(node, request)->sum`
  // TODO(wjwwood): consider error condition
  int i=0 ;
  for(i=0 ; i < 100 ; i++){
        request->a = i+1000;
  	auto result = send_request(node, client, request);
  	if (result) {
    		printf("[%d]Result of add_two_ints: %zd\n",i, result->sum);
  	} else {
    		printf("Interrupted while waiting for response. Exiting.\n");
  	}
   }

  rclcpp::shutdown();
  return 0;
}

