import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def load_urdf(pkg_name, urdf_file):
    pkg_path = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_path, 'urdf', urdf_file)
    with open(urdf_path, 'r') as f:
        return f.read()

def generate_launch_description():
    robot_desc1 = load_urdf('multi_urdf_display', 'cube.urdf')
    robot_desc2 = load_urdf('multi_urdf_display', 'sphere.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='cube_state_pub',
            namespace='cube',
            parameters=[{'robot_description': robot_desc1}],              
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='sphere_state_pub',
            namespace='sphere',
            parameters=[{'robot_description': robot_desc2}],
            
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='cube',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'cube/base_link'],
           
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='sphere',
            arguments=['2', '0', '0', '0', '0', '0', 'world', 'sphere/base_link'],
            
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('multi_urdf_display'),
                'rviz','display_multi_urdf.rviz')]
        )
    ])