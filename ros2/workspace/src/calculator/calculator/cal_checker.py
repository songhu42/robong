import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from user_interface.msg import CalArguments
from user_interface.action import CalChecker
from rclpy.action.client import ClientGoalHandle
import sys 
import random
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from asyncio import Future
from action_msgs.msg import GoalStatus
import argparse

class CalCheckerClient(Node):
    def __init__(self):
        super().__init__("cal_checker")
        
        self.declare_parameter("req_step", "+")
        self.add_on_set_parameters_callback(self.update_parameter)
        self.req_step = self.get_parameter("req_step").value

        self.action_client = ActionClient(self, CalChecker, "cal_checker")
        
        self.count = 0


    def send_goal(self, step):
        self.get_logger().info(f"request goal : {step}")

        while not self.action_client.wait_for_server():
            self.count += 1
            self.get_logger().info(f"server not availible : {self.count}")

        goal = CalChecker.Goal()
        if type('str') is str:
            goal.req_step = int(step)
        else :
            goal.req_step = step

        self.future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback )  
        self.future.add_done_callback(self.result_callback)
        
    
    def feedback_callback(self, msg):
        feedback_formula = msg.feedback.feedback_formula
        self.get_logger().info(f"feedback_formula : {feedback_formula}")

    def result_callback(self, future:Future):
        goal_handle : ClientGoalHandle= future.result()

        if not goal_handle.accepted:
            self.get_logger().info(f"goal is not accepted")
            return; 

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future:Future):
        goal_handle = future.result()
        status = goal_handle.status
        self.get_logger().info(f"get_result_callback status : {status}")
        result = goal_handle.result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"get_result_callback resall_formula : {result.all_formula}")
            self.get_logger().info(f"get_result_callback result sum : {result.result_res}")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"get_result_callback result canceled : {result.all_formula}")
        elif status == GoalStatus.STATUS_CANCELING:
            self.get_logger().info(f"get_result_callback result canceling : {result.all_formula}")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f"get_result_callback result aborted : {result.all_formula}")
        elif status == GoalStatus.STATUS_ACCEPTED:
            self.get_logger().info(f"get_result_callback result accepted .. ")
        elif status == GoalStatus.STATUS_EXECUTING:
            self.get_logger().info(f"get_result_callback result executing ....")

        raise SystemExit           

    def update_parameter(self, params):
        for param in params:
            if param.name == "req_step":
                self.req_step = param.value
                self.send_goal(self.req_step)
        
        print(f"update parameter oper : {self.req_step}")

        return SetParametersResult(successful=True)
    
def main(args=None):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-g',
        '--goal_steps',
        type=int,
        default=50,
        help= "Target goal value of calculate steps"
    )
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable'
    )
    args = parser.parse_args()

    rclpy.init(args=args.argv)
    node = CalCheckerClient()
    try:
        node.send_goal(args.goal_steps)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    except SystemExit:
        node.destroy_node()

if __name__ == "__main__":
    main()
