cmake_minimum_required(VERSION 3.8)
project(user_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)

# added 
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
  "msg/UserInts.msg" 
  "srv/AddAndOdd.srv"
  "action/Fibonacci.action"
  "msg/CalArguments.msg" 
  "srv/CalOperate.srv"
  "action/CalChecker.action"
  DEPENDENCIES builtin_interfaces) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# added 
ament_export_dependencies(rosidl_default_runtime)

ament_package()
