
ROS2_PATH=${HOME}/ros2_ws/
install_include=${ROS2_PATH}/install_isolated

all:_zoro_utils.so _zoro_utils_cpp.o _zoro_utils_cpp.a

zoro_utils_test:_zoro_utils_cpp.o _zoro_utils_test.o
	g++ $^ -o $@ -L${install_include}/rclcpp/lib -lrclcpp -llog4cxx

_zoro_utils.so:zoro_utils.o
	g++ -shared -fPIC $< -o $@ `python3.5-config --ldflags` -L${install_include}/rclcpp/lib -lrclcpp -llog4cxx

zoro_utils.o:_zoro_utils.cpp
	g++  -c $< -o $@ -fPIC -std=c++11 `python3.5-config --cflags` `python3.5-config --libs` -I${install_include}//rclcpp/include -I${install_include}//rcl/include -I${install_include}//rcutils/include -I${install_include}//rmw/include -I${install_include}//rosidl_generator_c/include -I${install_include}//rosidl_typesupport_interface/include -I${install_include}//rosidl_generator_cpp/include -I${install_include}//rcl_interfaces/include -I${install_include}//builtin_interfaces/include

_zoro_utils_cpp.o:_zoro_utils_cpp.cpp 
	g++  -c $< -o $@ -std=c++11 -I${install_include}//rclcpp/include -I${install_include}//rcl/include -I${install_include}//rcutils/include -I${install_include}//rmw/include -I${install_include}//rosidl_generator_c/include -I${install_include}//rosidl_typesupport_interface/include -I${install_include}//rosidl_generator_cpp/include -I${install_include}//rcl_interfaces/include -I${install_include}//builtin_interfaces/include

_zoro_utils_test.o:_zoro_utils_test.cpp
	g++ -c $< -o $@

_zoro_utils_cpp.a:_zoro_utils_cpp.o
	ar -rc $@ $< 

clean:
	rm *.o *.so *.a
