import numpy as np
import os
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String 
from rclpy.qos import QoSProfile 
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data as qos
from rclpy.clock import ClockType, Clock
from std_msgs.msg import Header 
import hw_sub_util as sub

def main(args=None):
    rclpy.init(args=args)
    msg_node = sub.Msg_sub("message2")
    try :
        rclpy.spin(msg_node)
    except KeyboardInterrupt:
        msg_node.get_logger().info("Keyboard interrupted!!")
    finally:
        msg_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    print('ROS2 Homework1018 Package hw_sub3')
    main()

    