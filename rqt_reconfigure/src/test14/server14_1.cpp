#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "example_interfaces/srv/add_two_ints.hpp"
//#include "my_interfaces/srv/add_two_ints.hpp"


class ServerNode : public rclcpp::Node
{
public:
  explicit ServerNode(const std::string & service_name)
  : Node("add_two_ints_server14_1")
  {
    // Create a callback function for when service requests are received.
    auto handle_add_two_ints =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) -> void
      {
        (void)request_header;
        printf( "Incoming request\na: %d , b: %d \n",
          int(request->a), int(request->b));
        response->sum = request->a + request->b;
        //sleep(1);
      };

    // Create a service that will use the callback function to handle requests.
    srv_ = create_service<example_interfaces::srv::AddTwoInts>(service_name, handle_add_two_ints);
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto service_name = std::string("add_two_ints_test");

  auto node = std::make_shared<ServerNode>(service_name);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

