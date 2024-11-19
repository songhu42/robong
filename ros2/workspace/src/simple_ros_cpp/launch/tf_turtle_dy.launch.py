import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 외부 런치 파일을 호출
    bringup_dir = get_package_share_directory('turtle_tf2_py')
    launch_dir = os.path.join(bringup_dir, 'launch')
    return LaunchDescription([

        # ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turtle_tf2_demo.launch.py'))),

        # ros2 run simple_ros_cpp dynamic_tf  
        Node(package="simple_ros_cpp", executable="dynamic_tf"),

        # ros2 run turtlesim dynamic_tf  
        Node(package="turtlesim", executable="turtlesim_node"),

        # rviz2
        Node(package="rviz2", executable="rviz2", output="screen"),
        
        # ros2 run tf2_ros static_transform_publisher --x 1 --y 1 --z 1 --yaw 0 --pitch 0 --roll 0 --frame-id world --child-frame-id static_tf_3
        Node(package="tf2_ros",
             executable="static_transform_publisher",
             arguments=['--x', '1', '--y', '1', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'static_tf_3']),
       
        ])
