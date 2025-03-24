# ros2_package

## tutorial 

### so_arm100_moveit

[【🚀SO-Arm100 机械臂 ROS2 配置保姆级教程】MoveIt Setup Assistant 从 0 到 1 实操！避坑指南](https://www.bilibili.com/video/BV1W3QWY5EUf/?vd_source=5ba34935b7845cd15c65ef62c64ba82f)

### display_so_arm100_rviz

[ROS2导入机械臂URDF竟有这些坑？SO-ARM100 Rivz可视化避坑指南](https://www.bilibili.com/video/BV1M8QLYFE3G?vd_source=5ba34935b7845cd15c65ef62c64ba82f&spm_id_from=333.788.videopod.sections)

### display_urdf_launch

[想知道两轮差速方形底盘 URDF 咋做，ROS2 配 Rviz 咋显示吗？看这里](https://www.bilibili.com/video/BV1Cq9NYhEiz?vd_source=5ba34935b7845cd15c65ef62c64ba82f&spm_id_from=333.788.videopod.sections)

### panda_moveit_controller

[一文搞定！ROS2 中用 MoveIt2 精准操控 Panda 机械臂末端至固定位姿](https://www.bilibili.com/video/BV1cY9HYuEMH?vd_source=5ba34935b7845cd15c65ef62c64ba82f&spm_id_from=333.788.videopod.sections)

[Panda换成SO-Arm100机械臂，代码仅换一行，末端固定位姿一样控制](https://www.bilibili.com/video/BV1XJQPYTEBH/?vd_source=5ba34935b7845cd15c65ef62c64ba82f)

### panda_joint_control

[ROS2 应用：按键控制 MoveIt2 中 Panda 机械臂关节位置](https://www.bilibili.com/video/BV1NAPueqEQL?vd_source=5ba34935b7845cd15c65ef62c64ba82f&spm_id_from=333.788.videopod.sections)

### yolo_target_detection

[ROS2下编写package利用orbbec相机进行yolov8实时目标检测](https://www.bilibili.com/video/BV1MCPFeEELG?vd_source=5ba34935b7845cd15c65ef62c64ba82f&spm_id_from=333.788.videopod.sections)

### tf_test_pkg

[ROS2 中 TF 变换发布与订阅：实现 base_link 和 test_link 实时可视化显示](https://www.bilibili.com/video/BV1tNADeWE1R?vd_source=5ba34935b7845cd15c65ef62c64ba82f&spm_id_from=333.788.videopod.sections)

### orbbec_cam_pkg

[ROS2下Rviz显示orbbec相机depth深度图](https://www.bilibili.com/video/BV1wwwfeeEtY?vd_source=5ba34935b7845cd15c65ef62c64ba82f&spm_id_from=333.788.videopod.sections)

[ROS2下编写orbbec相机C++ package并Rviz显示](https://www.bilibili.com/video/BV1HUwSe3E6o?vd_source=5ba34935b7845cd15c65ef62c64ba82f&spm_id_from=333.788.videopod.sections)

### mtc_tutorail

[ROS2 机械臂 MoveIt 开发必看！用 MTC 实现抓取任务规划的完整流程](https://www.bilibili.com/video/BV1JC9UYjErK/)

### moveit_servo_panda

[ROS2 Moveit2 手把手教你 Panda 机械臂笛卡尔速度控制!（附代码）](https://www.bilibili.com/video/BV1RUX7YmEBE/?vd_source=5ba34935b7845cd15c65ef62c64ba82f)

### joint_state_subscriber

[ROS2下MoveIt+Rviz+MuJoCo 三剑合璧！Panda 机械臂联动仿真！](https://www.bilibili.com/video/BV1GEo2YiERx/?vd_source=5ba34935b7845cd15c65ef62c64ba82f)

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

- yolo dependency

```
pip3 install ultralytics
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-image-transport
```