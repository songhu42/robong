import numpy as np
import os
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String 
from rclpy.qos import QoSProfile 
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data as qos

class M_sub(Node):
    def __init__(self):
        super().__init__('massage_subscriber')
        # self.qos_profile = QoSProfile(depth=10)
        # self.qos_profile = qos 
        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL, 
                                      reliability=QoSReliabilityPolicy.RELIABLE, 
                                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        self.massage_subscriber = self.create_subscription(String, 'massage', self.sub_topic_msg, self.qos_profile)
        

    def sub_topic_msg(self, msg):
        print(msg)
        self.get_logger().info(f"Received message : {msg.data}") 

def main(args=None):
    rclpy.init(args=args)
    node = M_sub()
    try :
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupted!!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    print('Hello ROS2 Simple Package subscribe')
    main()

    