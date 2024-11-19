from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="rqt_example", executable="examples", name="examples", output="screen", 
        namespace="turtle1"), 
        Node(package="turtlesim", executable="turtlesim_node", name="turtlesim", output="screen")
        ])