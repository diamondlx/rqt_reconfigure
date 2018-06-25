//created by ParameterGenerator
#ifndef __test__TUTORIALSCONFIG_H__
#define __test__TUTORIALSCONFIG_H__

#include<vector>

class ConfigureVec {
public:
  ConfigureVec() {
    ParameterVariantVec = std::vector<rclcpp::parameter::ParameterVariant> ({
      rclcpp::parameter::ParameterVariant("name.int_param","int_param"),
      rclcpp::parameter::ParameterVariant("des.int_param","An Integer parameter"),
      rclcpp::parameter::ParameterVariant("type.int_param","int"),
      rclcpp::parameter::ParameterVariant("lev.int_param",0),
      rclcpp::parameter::ParameterVariant("min.int_param",0),
      rclcpp::parameter::ParameterVariant("max.int_param",100),
      rclcpp::parameter::ParameterVariant("default.int_param",50),
      rclcpp::parameter::ParameterVariant("value.int_param",50),
      rclcpp::parameter::ParameterVariant("name.double_param","double_param"),
      rclcpp::parameter::ParameterVariant("des.double_param","A double parameter"),
      rclcpp::parameter::ParameterVariant("type.double_param","double"),
      rclcpp::parameter::ParameterVariant("lev.double_param",0),
      rclcpp::parameter::ParameterVariant("min.double_param",0.0),
      rclcpp::parameter::ParameterVariant("max.double_param",1.0),
      rclcpp::parameter::ParameterVariant("default.double_param",0.5),
      rclcpp::parameter::ParameterVariant("value.double_param",0.5),
      rclcpp::parameter::ParameterVariant("name.str_param","str_param"),
      rclcpp::parameter::ParameterVariant("des.str_param","A string parameter"),
      rclcpp::parameter::ParameterVariant("type.str_param","str"),
      rclcpp::parameter::ParameterVariant("lev.str_param",0),
      rclcpp::parameter::ParameterVariant("min.str_param",""),
      rclcpp::parameter::ParameterVariant("max.str_param",""),
      rclcpp::parameter::ParameterVariant("default.str_param","Hello World"),
      rclcpp::parameter::ParameterVariant("value.str_param","Hello World"),
      rclcpp::parameter::ParameterVariant("name.bool_param","bool_param"),
      rclcpp::parameter::ParameterVariant("des.bool_param","A Boolean parameter"),
      rclcpp::parameter::ParameterVariant("type.bool_param","bool"),
      rclcpp::parameter::ParameterVariant("lev.bool_param",0),
      rclcpp::parameter::ParameterVariant("min.bool_param",false),
      rclcpp::parameter::ParameterVariant("max.bool_param",true),
      rclcpp::parameter::ParameterVariant("default.bool_param",true),
      rclcpp::parameter::ParameterVariant("value.bool_param",true),
      rclcpp::parameter::ParameterVariant("name.size","size"),
      rclcpp::parameter::ParameterVariant("des.size","A size parameter which is edited via an enum"),
      rclcpp::parameter::ParameterVariant("type.size","int"),
      rclcpp::parameter::ParameterVariant("lev.size",0),
      rclcpp::parameter::ParameterVariant("min.size",0),
      rclcpp::parameter::ParameterVariant("max.size",3),
      rclcpp::parameter::ParameterVariant("default.size",1),
      rclcpp::parameter::ParameterVariant("value.size",1),
    });
  }
  std::vector<rclcpp::parameter::ParameterVariant> getParameterVariantVec() {
    return ParameterVariantVec;
  }
private:
std::vector<rclcpp::parameter::ParameterVariant> ParameterVariantVec;
};
#endif
