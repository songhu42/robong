import numpy as np
import os
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String 
from rclpy.qos import QoSProfile 
from rclpy.clock import ClockType, Clock
from std_msgs.msg import Header 
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data as qos

class Msg_pub(Node):
    def __init__(self):
        super().__init__('massage_publisher')
        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL, 
                                reliability=QoSReliabilityPolicy.RELIABLE, 
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.massage_publisher = self.create_publisher(String, 'massage2', self.qos_profile)
        self.timer = self.create_timer(1, self.m_publisher)
        self.count = 0

    def m_publisher(self):
        msg = String()
        msg.data = f"hello messaage2 => {self.count}"
        self.massage_publisher.publish(msg)
        self.get_logger().info(f"P:ushished message : {msg.data}")
        self.count += 1


# Header 발행 .. 
class Header_pub(Node):
    def __init__(self):
        super().__init__('header_publisher')
        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                      durability=QoSDurabilityPolicy.VOLATILE,
                                      depth=10)
        self.create_timer(1, self.print_hello)
        self.pub = self.create_publisher(Header, "time", self.qos_profile)
        # self.clock = self.get_clock()
        # ROS_TIME, STEADY_TIME, SYSTEM_TIME
        self.clock = Clock(clock_type=ClockType.ROS_TIME)


    def print_hello(self):
        msg = Header()
        msg.frame_id = "time"
        msg.stamp = self.clock.now().to_msg()
        print(f"sec: {msg.stamp.sec}, nano sec : {msg.stamp.nanosec}")
        self.pub.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    msg_node = Msg_pub()
    header_node = Header_pub()
    try :
        rclpy.spin(msg_node)
        rclpy.spin(header_node)
    except KeyboardInterrupt:
        msg_node.get_logger().info("Keyboard interrupted!!")
        header_node.get_logger().info("Keyboard interrupted!!")
    finally:
        msg_node.destroy_node()
        header_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    print('ROS2 Homework1018 Package hw_pub')
    main()

    