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
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from user_interface.action import CalChecker

# callback_group : 같은 함수를 여러번 호출하는 경우에만 적용됨..
from rclpy.callback_groups import ReentrantCallbackGroup
# 순차적으로 실행해야 할때.. 
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


import time 

class Calculator(Node):
    def __init__(self):
        super().__init__("calculator")
        
        # 1. argument change sub 
        self.create_subscription(CalArguments, "send_argument", self.argument_sub_callback, 10)
        self.declare_parameter("var_a", 0)
        self.declare_parameter("var_b", 100)
        self.add_on_set_parameters_callback(self.update_parameter)
        self.var_a = self.get_parameter("var_a").value
        self.var_b = self.get_parameter("var_b").value

        # self.create_timer(1, self.print_info)

        self.callback_group = ReentrantCallbackGroup()

        # 2. service server
        self.oper = "+"
        self.service_server = self.create_service(CalOperate, "operateService", 
                                                  self.operate_callback, callback_group=self.callback_group) 

        # 3. action server : 계산 횟수만큼 계산 식 전송 후 종료한다. 
        self.action_server = ActionServer(self, CalChecker, "cal_checker", self.cal_checker_callback, callback_group=self.callback_group)
        self.res = 0
        self.fomul = ""
        self.count = 0

    def argument_sub_callback(self, msg: CalArguments):
        self.get_logger().info(f"argument_sub_callback stamp : {msg.stamp} var_a : {msg.var_a} var_b : {msg.var_b}")
        self.var_a = msg.var_a
        self.var_b = msg.var_b

    def calculate(self):
        temp_fomul = str(self.var_a) + " " + self.oper + " " + str(self.var_b) + " = "
        temp_res = 0.0 
        if self.oper == "+": 
            temp_res = self.var_a + self.var_b
        elif self.oper == "-": 
            temp_res = self.var_a - self.var_b
        elif self.oper == "*": 
            temp_res = self.var_a * self.var_b
        elif self.oper == "/": 
            temp_res = self.var_a / self.var_b
        else :
            temp_fomul = "Wrong operator " + self.oper
            temp_res = 0.0

        self.fomul = temp_fomul + str(temp_res)
        self.res = temp_res
    
    def random_calculate(self) :
        self.var_a = random.randint(1, 100)
        self.var_b = random.randint(1, 100)
        oper_int = random.randint(0, 4)
        if oper_int == 0 : 
            self.oper = "+"
        elif oper_int == 1:
            self.oper = "-"
        elif oper_int == 2:
            self.oper = "*"
        elif oper_int == 3:
            self.oper = "/"

        self.calculate()

    def operate_callback(self, request:CalOperate.Request, response:CalOperate.Response):
        self.get_logger().info(f"request data : {request.oper}")
        self.oper = request.oper

        self.calculate()

        response.fomul = self.fomul
        response.res = self.res

        self.get_logger().info(f"response {self.count} => {response.fomul}")

        return response

    def cal_checker_callback(self, goal_handle:ServerGoalHandle):
        self.get_logger().info(f"request step : {goal_handle.request.req_step}")

        goal = CalChecker.Goal()
        result = CalChecker.Result()
        feedback = CalChecker.Feedback()

        result_res = 0.0

        req_step = goal_handle.request.req_step

        if req_step <= 0:
            self.get_logger().info(f"req_step is less than 1 : {req_step}")
            # when server error will be return STATUS_ABORTED
            goal_handle.abort()
            result.feedback_res = []
            result.result_res = result_res 

            return result
        else:

            for i in range(1, req_step+1):
                # 전체 결과값 sum return to result 
                self.random_calculate()

                result_res += self.res 
                feedback.feedback_formula = self.fomul
                
                feedback.feedback_res.append(self.res)
                result.all_formula.append(self.fomul)
                # send feedback msg 
                goal_handle.publish_feedback(feedback)
                time.sleep(1)

            goal_handle.succeed()
            result.feedback_res = feedback.feedback_res
            result.result_res = result_res 
            
            return result
            
    def update_parameter(self, params):
        for param in params:
            if param.name == "var_a":
                self.var_a = param.value
            elif param.name == "var_b":
                self.var_b = param.value
        
        print(f"update parameter var_a : {self.var_a} var_b : {self.var_b}")
        SetParametersResult()
        return SetParametersResult(successful=True)
    
def main():
    rclpy.init()
    try:
        # Multi Thread 
        executor = MultiThreadedExecutor(num_threads=10)
        node = Calculator()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.destroy_node()
        executor.shutdown()

if __name__ == "__main__":
    main()
