cmake_minimum_required(VERSION 3.0)
project(hello)

find_package(OpenCV, REQUIRED)
add_executable(main hello.cpp)

# find_package 이름 + '_LIB_DIR' 
target_link_libraries(main, ${OpenCV_LIB_DIR})

target_link_directories(main, PUBLIC ${OpenCV_LIB_DIR})



