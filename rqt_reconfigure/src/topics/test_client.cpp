#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <vector>

class Client {
public:
  Client(const rclcpp::Node::SharedPtr node, const std::string remote_name) : parameters_client(node, remote_name) {
    printf("Client() 0\n");
    //auto node = rclcpp::Node::make_shared("get_parameters_test_client");
    printf("Client() 1\n");
    //parameters_client = rclcpp::SyncParametersClient(node, "get_parameters3");
    printf("Client() 2\n");
    auto listdata = parameters_client.list_parameters({"name"}, 0);
    printf("2\n");
    for(auto name : listdata.names){
      std::cout << name << std::endl;
    //std::cout << "name." + name << std::endl;
      std::string subs = name.substr(5);

      name_vec.push_back(name.substr(5));

      auto param_des = parameters_client.get_parameters({
        "name." + subs,
        "type." + subs, 
        "default." + subs, 
        "max." + subs, 
        "min." + subs, 
        "value." + subs, 
        "des." + subs,
        "lev." + subs,
      });
      std::cout << param_des << std::endl;
      //param_des.gg();
    //for (auto )
    }
    printf("3\n");
    printf("vec\n");
    for (std::string name : name_vec) {
      std::cout << name << std::endl;
    }
  }

  std::vector<rclcpp::parameter::ParameterVariant> get_description(){
    std::vector<rclcpp::parameter::ParameterVariant> description;
    for (std::string name : name_vec) {
      std::cout << name << std::endl;
      auto param_des = parameters_client.get_parameters({
        "name." + name,
        "type." + name, 
        "default." + name, 
        "max." + name, 
        "min." + name, 
        "value." + name, 
        "des." + name,
        "lev." + name,
      });
      description.insert(description.end(), param_des.begin(), param_des.end());
    }
    std::cout << description << std::endl;
    return description;
  }
  std::vector<rclcpp::parameter::ParameterVariant> get_values(){
    
    std::vector<rclcpp::parameter::ParameterVariant> values;
    for (std::string name : name_vec) {
      std::cout << name << std::endl;
      auto param_des = parameters_client.get_parameters({
        "value." + name,
      });
      values.insert(values.end(), param_des.begin(), param_des.end());
    }
    std::cout << values << std::endl;
    return values;
  }
  //void get_
private:
  rclcpp::SyncParametersClient parameters_client;
  std::vector<std::string> name_vec;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("get_parameters4");
  printf("000\n");
  //auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "get_parameters3");
  Client test_client = Client(node, "get_parameters3");
  printf("1\n");
  rclcpp::shutdown();
  return 0;
}