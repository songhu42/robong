cmake_minimum_required(VERSION 3.0)
project(hello)

find_package(OpenCV REQUIRED)

# 소스 읽어서 파일별 처리함.. 
file(GLOB SOURCES "*.cpp")

foreach(SOURCE ${SOURCES})
    message(STATUS "SOURCE: ${SOURCE}" )
    
    get_filename_component(EXE_NAME ${SOURCE} NAME_WE)

    message(STATUS "EXE_NAME: ${EXE_NAME}" )

    add_executable(${EXE_NAME} ${SOURCE})

    # find_package 이름 + '_LIB_DIR' 
    target_link_libraries(${EXE_NAME} ${OpenCV_LIBS})
    target_link_directories(${EXE_NAME} PUBLIC ${OpenCV_LIB_DIR})
endforeach()


