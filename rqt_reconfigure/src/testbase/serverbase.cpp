#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <gflags/gflags.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "example_interfaces/srv/add_two_ints.hpp"

DEFINE_string(name, "server", "the node name ");
DEFINE_string(topic, "add_two_int", "the service topic name ");
DEFINE_bool(isprint, false, "whether print log ");
DEFINE_int32(sleeptime, 0, "sleep time");


class ServerNode : public rclcpp::Node
{
public:
  explicit ServerNode(const std::string  service_name, const std::string node_name, const bool isprint_, const size_t sleeptime)
  : Node(node_name)
  {
    // Create a callback function for when service requests are received.
    auto handle_add_two_ints =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) -> void
      {
        (void)request_header;
        if(isprint)
          printf( "Incoming request\na: %d , b: %d \n",int(request->a), int(request->b));
        response->sum = request->a + request->b;
        sleep(sleep_time);
      };

    // Create a service that will use the callback function to handle requests.
    srv_ = create_service<example_interfaces::srv::AddTwoInts>(service_name, handle_add_two_ints);
    isprint = isprint_;
    sleep_time = sleeptime;
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
  bool isprint;
  size_t sleep_time;
};

long get_delay(struct timespec &end, struct timespec &start)
{
  return (end.tv_sec - start.tv_sec)* 1000 + (end.tv_nsec - start.tv_nsec)/1000000 ;
}

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  struct timespec ts_start;
  clock_gettime( CLOCK_REALTIME , &ts_start);

  std::string usage("\tthis program is a base server program used for ros2_zoro test");
  google::SetUsageMessage(usage);
  google::ParseCommandLineFlags(&argc, &argv, true);

  rclcpp::init(argc, argv);

  struct timespec ts_init;
  clock_gettime( CLOCK_REALTIME , &ts_init);
  if(FLAGS_isprint)
    std::cout<<"init time is : "<< get_delay(ts_init,ts_start)<< " ms" << std::endl;

  auto node = std::make_shared<ServerNode>(FLAGS_topic, FLAGS_name, FLAGS_isprint, FLAGS_sleeptime);

  struct timespec ts_prepare;
  clock_gettime( CLOCK_REALTIME , &ts_prepare);
  if(FLAGS_isprint)
    std::cout<<" node creation time is : "<< get_delay(ts_prepare,ts_start) << " ms" << std::endl;

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

