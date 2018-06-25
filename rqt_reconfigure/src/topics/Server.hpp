#ifndef __rqt__reconfigure__SERVER_H__
#define __rqt__reconfigure__SERVER_H__


#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "rclcpp/macros.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace rqt_reconfigure {

template <class ConfigType> 
class Server {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Server)
  
  RCLCPP_PUBLIC
  explicit Server(const rclcpp::Node::SharedPtr node_,
      std::function<void(std::vector<rclcpp::parameter::ParameterVariant>)> user_callback) :
      node(node_), callback_function(user_callback) {
    printf("Server init____________\n");
    parameter_service = std::make_shared<rclcpp::ParameterService>(node);
    //auto parameters_init_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    node->set_parameters(ConfigType().getParameterVariantVec());
    auto listdata = node->list_parameters({"name"}, 0);
    for(auto name : listdata.names) {
      auto param_val = node->get_parameters({
        "value." + name.substr(5)
      });
      values.insert(values.end(), param_val.begin(), param_val.end());
    }
    auto callback =
      [this](const std::vector<rclcpp::parameter::ParameterVariant> & parameter_changed) 
        -> rcl_interfaces::msg::SetParametersResult
      { 
        this->param_change(parameter_changed);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = 1;
        return result;
      };
    node->register_param_change_callback(callback);
  }
  void param_change(const std::vector<rclcpp::parameter::ParameterVariant> &parameter_changed) {
    for (auto iter = values.begin(); iter != values.end();) {
      if(iter->get_name() == parameter_changed[0].get_name()) {
        iter = values.erase(iter);
      }
      else
        iter ++;
    }
    values.insert(values.end(), parameter_changed.begin(), parameter_changed.end());
    std::cout << values << std::endl;
    callback_function(values);
  }
  void set_callback(std::function<void(std::vector<rclcpp::parameter::ParameterVariant>)> callback_f) {
    callback_function = callback_f;
  }
private:
  std::shared_ptr<rclcpp::ParameterService> parameter_service;
  rclcpp::Node::SharedPtr node;
  std::vector<rclcpp::parameter::ParameterVariant> values;
  std::function<void(std::vector<rclcpp::parameter::ParameterVariant>)> callback_function;
};

}//rqt_reconfigure

#endif 