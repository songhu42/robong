import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from user_interface.msg import CalArguments
import random
from rcl_interfaces.msg import SetParametersResult

class ArgumentPub(Node):
    def __init__(self):
        super().__init__("argument_pub")
        
        self.declare_parameter("min", 0)
        self.declare_parameter("max", 100)
        self.add_on_set_parameters_callback(self.update_parameter)
        self.min = self.get_parameter("min").value
        self.max = self.get_parameter("max").value

        self.create_timer(1, self.argument_send)
        self.pub = self.create_publisher(CalArguments, "send_argument", 10)
        self.count = 0

    def argument_send(self):
        msg = CalArguments()
        msg.stamp = self.get_clock().now().to_msg()
        msg.var_a = float(random.randint(self.min, self.max))
        msg.var_b = float(random.randint(self.min, self.max))
        
        self.pub.publish(msg)
        self.get_logger().info(f"argument_pub var_a : {msg.var_a} var_b : {msg.var_b}")
        self.count += 1

    def update_parameter(self, params):
        for param in params:
            if param.name == "min":
                self.min = param.value
            elif param.name == "max":
                self.max = param.value
        
        self.get_logger().warn(f"update parameter min : {self.min} max : {self.max}")

        return SetParametersResult(successful=True)
    
def main():
    rclpy.init()
    node = ArgumentPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()
