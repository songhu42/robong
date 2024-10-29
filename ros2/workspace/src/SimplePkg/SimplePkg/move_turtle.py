import numpy as np
import os
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String 
from rclpy.qos import QoSProfile 
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data as qos
from turtlesim.msg import Color, Pose, Twist

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')
        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, 
                                      reliability=QoSReliabilityPolicy.BEST_EFFORT, 
                                      durability=QoSDurabilityPolicy.VOLATILE,
                                      depth=10)   
        
        self.create_timer(0.1, self.twist_pub)
        self.create_timer(1/60, self.update)

        self.massage_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', self.qos_profile)
        self.create_subscription(Pose, "turtle1/pose", self.pose_callback)
        self.create_subscription(Color, "turtle1/color_sensor", self.color_callback)

        self.count = 0
        self.twist = Twist()
        self.pose = Pose()
        self.color = Color()

    def twist_pub(self):
        self.pub.publish(self.twist)

    def pose_callback(self, msg:Pose):
        self.pose = msg

    def color_callback(self, msg:Color):
        self.color = msg

    def print_hello(self):
        self.get_logger().info(f"print_hello message..")
        self.count += 1

    def update(self):
        self.twist.linear.x += 0.001
        self.twist.angular.z = 1.0

def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtle()
    try :
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupted!!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    print('Hello ROS2 Simple Package second')
    main()

    