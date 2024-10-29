import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from user_interface.msg import CalArguments
from user_interface.srv import CalOperate
import random
from rcl_interfaces.msg import SetParametersResult

class OperatorClient(Node):
    def __init__(self):
        super().__init__("operator_client")
        
        self.declare_parameter("oper", "+")
        self.add_on_set_parameters_callback(self.update_parameter)
        self.oper = self.get_parameter("oper").value
        
        self.declare_parameter("service_time", 2)
        self.service_time = self.get_parameter("service_time").get_parameter_value().integer_value

        self.client = self.create_client(CalOperate, "operateService") 
        while not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("service operateService is not available!!")

        self.request = CalOperate.Request()

        self.create_timer(self.service_time, self.send_request)
        self.count = 0
        self.isSetted = False

    def send_request(self):
        
        # 외부에서 operator 변경된 경우 random으로 바꾸지 않는다. 
        if not self.isSetted:
            # random operator select 
            oper_int = random.randint(0, 4)
            if oper_int == 0 : 
                self.oper = "+"
            elif oper_int == 1:
                self.oper = "-"
            elif oper_int == 2:
                self.oper = "*"
            elif oper_int == 3:
                self.oper = "/"

        self.request.oper = self.oper

        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        response : CalOperate.Response = future.result()
        self.get_logger().info(f"calculate fomular : {response.fomul}")
        self.get_logger().info(f"calculate result = {response.res}")

    def update_parameter(self, params):
        for param in params:
            if param.name == "oper":
                self.isSetted = True
                self.oper = param.value
                self.send_request()
        
        print(f"update parameter oper : {self.oper}")

        return SetParametersResult(successful=True)
    
def main():
    rclpy.init()
    node = OperatorClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()
