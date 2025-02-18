## create template
```
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake usb_cam_pkg --dependencies rclcpp sensor_msgs cv_bridge image_transport OpenCV
```

## CMakeLists.txt
```
add_executable(usb_cam_node src/usb_cam_node.cpp)
ament_target_dependencies(usb_cam_node rclcpp sensor_msgs cv_bridge image_transport OpenCV)

install(TARGETS
  usb_cam_node
  DESTINATION lib/${PROJECT_NAME}
)
```

## build
```
cd ~/ros2_ws
colcon build --packages-select usb_cam_pkg
```

## run
```
source install/setup.bash
ros2 run usb_cam_pkg usb_cam_node
```