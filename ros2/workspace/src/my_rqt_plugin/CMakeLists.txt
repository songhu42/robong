cmake_minimum_required(VERSION 3.8)
project(my_rqt_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclpy REQUIRED)
#find_package(rqt_gui REQUIRED)
#find_package(rqt_gui_py REQUIRED)
#find_package(python_qt_binding REQUIRED)

install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resource launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/rqt_example DESTINATION lib/${PROJECT_NAME}) 




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
