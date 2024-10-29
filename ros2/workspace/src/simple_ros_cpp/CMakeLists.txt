cmake_minimum_required(VERSION 3.8)
project(simple_ros_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# added by manual... 
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(user_interface REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(hello_pub src/hello_pub.cpp)
ament_target_dependencies(hello_pub rclcpp std_msgs) 

add_executable(hello_pub_class src/hello_pub_class.cpp src/hello_pub_class_method.cpp)
ament_target_dependencies(hello_pub_class rclcpp std_msgs) 

add_executable(hello_sub src/hello_sub.cpp src/hello_sub_method.cpp)
ament_target_dependencies(hello_sub rclcpp std_msgs) 

add_executable(hello_sub_cv src/hello_sub_cv.cpp src/hello_sub_cv_method.cpp)
target_link_libraries(hello_sub_cv ${OpenCV_LIBS})
target_link_directories(hello_sub_cv PUBLIC ${OpenCV_LIBS_DIR})
ament_target_dependencies(hello_sub_cv rclcpp std_msgs) 

add_executable(move_turtle src/move_turtle.cpp )
ament_target_dependencies(move_turtle rclcpp geometry_msgs turtlesim) 

add_executable(service_server src/service_server.cpp )
ament_target_dependencies(service_server rclcpp std_srvs) 
add_executable(service_client src/service_client.cpp )
ament_target_dependencies(service_client rclcpp std_srvs) 

add_executable(service_client_int src/service_client_int.cpp )
ament_target_dependencies(service_client_int rclcpp user_interface) 

add_executable(simple_param src/simple_param.cpp )
ament_target_dependencies(simple_param rclcpp rcl_interfaces) 

add_executable(action_server src/action_server.cpp )
ament_target_dependencies(action_server rclcpp user_interface rclcpp_action) 

set(NODE_NAME action_client)
add_executable(${NODE_NAME} src/${NODE_NAME}.cpp )
ament_target_dependencies(${NODE_NAME} rclcpp user_interface rclcpp_action) 

add_executable(logging_exam src/logging_exam.cpp)
ament_target_dependencies(logging_exam rclcpp std_msgs) 

add_executable(two_node_pipe src/two_node_pipe.cpp)
ament_target_dependencies(two_node_pipe rclcpp std_msgs) 

add_executable(two_node_cyclic_pipe src/two_node_cyclic_pipe.cpp)
ament_target_dependencies(two_node_cyclic_pipe rclcpp std_msgs) 

add_executable(image_pipeline_all src/image_pipeline_all.cpp)
target_link_libraries(image_pipeline_all
  rclcpp::rclcpp
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  opencv_core
  opencv_highgui)


install(TARGETS 
  hello_pub 
  hello_pub_class 
  hello_sub 
  hello_sub_cv 
  move_turtle
  service_server
  service_client
  service_client_int
  simple_param
  action_server
  action_client
  logging_exam
  two_node_pipe
  two_node_cyclic_pipe
  image_pipeline_all
  DESTINATION lib/${PROJECT_NAME})
# added by manual finish... 


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
