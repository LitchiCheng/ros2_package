# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motormanager_node = Node(
        package="motor_package",
        executable="motormanager"
    )
    motormanager_node1 = Node(
        package="motor_package",
        executable="motormanager"
    )
    vehicle_node = Node(
        package="vehicle_package",
        executable="vehicle"
    )
    # 可以灵活的添加几个node，比如
    nodes = []
    for i in range(10):
        node = Node(package="motor_package", executable="motormanager")
        nodes.append(node)
    vehicle_node1 = Node(
        package="vehicle_package",
        executable="vehicle"
    )
    nodes.append(vehicle_node1)
    launch_description = LaunchDescription(
        nodes)
    return launch_description
