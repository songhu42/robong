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

class Msg_sub(Node):
    def __init__(self, nodename):
        super().__init__('massage_subscriber')
        # self.qos_profile = QoSProfile(depth=10)
        # self.qos_profile = qos 
        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL, 
                                      reliability=QoSReliabilityPolicy.RELIABLE, 
                                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        self.massage_subscriber = self.create_subscription(String, nodename, self.sub_topic_msg, self.qos_profile)
        

    def sub_topic_msg(self, msg):
        print(msg)
        self.get_logger().info(f"Received message : {msg.data}") 


class Time_sub(Node):
    def __init__(self, nodename):
        super().__init__('time_subscriber')
        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL, 
                                      reliability=QoSReliabilityPolicy.RELIABLE, 
                                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        self.massage_subscriber = self.create_subscription(Header, nodename, self.sub_topic_msg, self.qos_profile)
        

    def sub_topic_msg(self, msg):
        print(f"sec: {msg.stamp.sec}, nano sec : {msg.stamp.nanosec}")

