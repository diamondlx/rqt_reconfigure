#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <gflags/gflags.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "example_interfaces/srv/add_two_ints.hpp"

DEFINE_string(name, "client", "the node name ");
DEFINE_string(topic, "add_two_int", "the service topic name ");
DEFINE_bool(isprint, false, "whether print log ");
DEFINE_int32(num, 1000, "one of number to be added");
DEFINE_int32(freq, 20, "the frequency of request");
DEFINE_int32(loop, 100, "the times of request");

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

long get_delay(struct timespec &end, struct timespec &start)
{
  return (end.tv_sec - start.tv_sec)* 1000 + (end.tv_nsec - start.tv_nsec)/1000000 ;
}
int main(int argc, char ** argv)
{
  struct timespec ts_start;
  clock_gettime( CLOCK_REALTIME , &ts_start);

  std::string usage("\tthis program is a base client program used for ros2_zoro test");
  google::SetUsageMessage(usage);
  google::ParseCommandLineFlags(&argc, &argv, true);

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  struct timespec ts_init;
  clock_gettime( CLOCK_REALTIME , &ts_init);
  if(FLAGS_isprint)
    std::cout<<"init time is : "<< get_delay(ts_init,ts_start)<< " ms" << std::endl;

  auto node = rclcpp::Node::make_shared(FLAGS_name);

  struct timespec ts_prepare;
  clock_gettime( CLOCK_REALTIME , &ts_prepare);
  if(FLAGS_isprint)
    std::cout<<" node creation time is : "<< get_delay(ts_prepare,ts_start) << " ms" << std::endl;

  auto topic = std::string(FLAGS_topic);
  auto client = node->create_client<example_interfaces::srv::AddTwoInts>(topic);

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = FLAGS_num;
  request->b = 1;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      printf( "Interrupted while waiting for the service. Exiting.\n");
      return 0;
    }
    if(FLAGS_isprint)
      printf( "service not available, waiting again...\n");
  }

  int i=0 ;
  for(i=0 ; i < FLAGS_loop ; i++){
        request->b = i;
  	auto result = send_request(node, client, request);
  	if (result) {
  	  if(FLAGS_isprint)
  	    printf("[%d]Result of add_two_ints: %zd\n",i, result->sum);
  	} else {
      if(FLAGS_isprint)
        printf("Interrupted while waiting for response. Exiting.\n");
  	}
   }

  rclcpp::shutdown();
  return 0;
}

