from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="SimplePkg", executable="hello_pub"), 
        Node(package="SimplePkg", executable="hello_sub")])

