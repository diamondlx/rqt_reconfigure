#ifndef __rqt__reconfigure__SERVER_H__
#define __rqt__reconfigure__SERVER_H__

#include <Python.h>
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "rclcpp/macros.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "testRclConfig.h"

namespace rqt_reconfigure {

class Server_py {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Server_py)
  
  RCLCPP_PUBLIC
  explicit Server_py(const std::string node_name, PyObject* python_callback_,
      const std::vector<rclcpp::parameter::ParameterVariant> &parameter_init) : python_callback(python_callback_) {
    printf("Server init____________\n");
    node = rclcpp::Node::make_shared(node_name);
    parameter_service = std::make_shared<rclcpp::ParameterService>(node);
    //auto parameters_init_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    node->set_parameters(parameter_init);
    
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
    rclcpp::spin(node);
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
    PyObject* dict=PyDict_New();
    for (auto parameter : values) {

      std::string name = parameter.get_name().data();
      name = name.substr(6);
      switch(parameter.get_type()){
        case rclcpp::parameter::ParameterType::PARAMETER_BOOL:
          if(parameter.as_bool()) {
            PyDict_SetItemString(dict, name.c_str(), Py_True);
          } else {
            PyDict_SetItemString(dict, name.c_str(), Py_False);
          }
          continue;
        case rclcpp::parameter::ParameterType::PARAMETER_INTEGER:
          PyDict_SetItemString(dict, name.c_str(), PyLong_FromLong(parameter.as_int()));
          continue;
        case rclcpp::parameter::ParameterType::PARAMETER_DOUBLE:
          PyDict_SetItemString(dict, name.c_str(), PyFloat_FromDouble(parameter.as_double()));
          continue;
        case rclcpp::parameter::ParameterType::PARAMETER_STRING:
          PyDict_SetItemString(dict, name.c_str(), PyUnicode_FromString(parameter.as_string().data()));
          continue;
        default:
          continue;
      }
    }

    PyObject* arglist = Py_BuildValue("(O)", dict);
    if (PyCallable_Check(python_callback)) {
      PyEval_CallObject(python_callback, arglist);
    }
  }
private:
  std::shared_ptr<rclcpp::ParameterService> parameter_service;
  rclcpp::Node::SharedPtr node;
  std::vector<rclcpp::parameter::ParameterVariant> values;
  PyObject* python_callback;
};

}//rqt_reconfigure

#endif 