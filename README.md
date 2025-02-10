# ros2_package

## build

```
# turn to workspace
cd ~/ws
# build all package
colcon build 
# build select package
colcon build --packages-select my_package
# source builded package
source install/setup.bash
```

## matters need attention
- running orbbec_cam_pkg is root permission required.

```
sudo su
source /opt/ros2/humble/setup.bash 
source install/setup.bash 
ros2 run orbbec_cam_pkg image_publisher

rviz2
```