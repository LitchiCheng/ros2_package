cmake_minimum_required(VERSION 3.8)
project(orbbec_cam_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB ORBBEC_LIBS "lib/x86_64/*")

include_directories(include)
include_directories(utility)

add_executable(image_publisher src/orbbec.cpp src/image_publisher.cpp)
ament_target_dependencies(image_publisher rclcpp sensor_msgs cv_bridge OpenCV)

target_link_libraries(image_publisher
    ${ORBBEC_LIBS}
)

install(TARGETS
  image_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  lib/x86_64/
  DESTINATION lib/)

ament_package()