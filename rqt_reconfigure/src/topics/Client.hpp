#ifndef __rqt__reconfigure__CLIENT_H__
#define __rqt__reconfigure__CLIENT_H__


#include "rclcpp/rclcpp.hpp"

namespace rqt_reconfigure {

class Client {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Client)
  
  RCLCPP_PUBLIC
  explicit Server(const rclcpp::Node::SharedPtr node) {
    printf("Server init\n");
    parameter_service = std::make_shared<rclcpp::ParameterService>(node);
    auto parameters_init_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    parameters_init_client->set_parameters(ConfigType().getParameterVariantVec());
    printf("Server init end\n");
  }
private:
  std::shared_ptr<rclcpp::ParameterService> parameter_service;
};

}//rqt_reconfigure

#endif 