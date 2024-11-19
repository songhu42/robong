import rclpy
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.clock import Duration
from rclpy.node import Node
from rclpy.task import Future


class Patrol_manipulator(Node):
    def __init__(self):
        super().__init__("patrol_manipulator")
        self.client = self.create_client(SetJointPosition, "goal_joint_space_path")
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")
        self.posi_req = SetJointPosition.Request()
        self.task_req = SetKinematicsPose.Request()

        self.create_timer(1/60, self.update)
        self.joint_angles = self.posi_req.joint_position.position 
        self.goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.prev_time = self.get_clock().now()
        self.stage = 0

    def send_request(self, path_time=0.5):
        # self.joint_angles = [0.0, 1.0, 1.0, -1.0, -1.0]
        self.posi_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        self.posi_req.joint_position.position = self.joint_angles
        self.posi_req.path_time = path_time

        self.future = self.client.call_async(self.posi_req)
        self.future.add_done_callback(self.done_callback)


    def send_goal_task_space(self, path_time=0.5):
        self.task_req.end_effector_name = 'gripper'
        self.task_req.kinematics_pose.pose.position.x = self.goal_kinematics_pose[0]
        self.task_req.kinematics_pose.pose.position.y = self.goal_kinematics_pose[1]
        self.task_req.kinematics_pose.pose.position.z = self.goal_kinematics_pose[2]
        self.task_req.kinematics_pose.pose.orientation.w = self.goal_kinematics_pose[3]
        self.task_req.kinematics_pose.pose.orientation.x = self.goal_kinematics_pose[4]
        self.task_req.kinematics_pose.pose.orientation.y = self.goal_kinematics_pose[5]
        self.task_req.kinematics_pose.pose.orientation.z = self.goal_kinematics_pose[6]
        self.task_req.path_time = path_time

        try:
            send_goal_task = self.goal_task_space.call_async(self.task_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))


    def done_callback(self, future : Future):
        response : SetJointPosition.Response = future.result()
        self.get_logger().info(f"done_callback : {response.is_planned}")

    def update(self):
        if self.prev_time + Duration(seconds=3) > self.get_clock().now():
            # first move
            self.joint_angles = [0.5, 0.0, 0.3, 0.0, 1.0]
            if self.stage == 0:
                print(f"stage 0 called")
                self.send_request(2.0)
                self.goal_kinematics_pose = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]
                self.send_goal_task_space(0.3)
                self.stage += 1
        elif self.prev_time + Duration(seconds=5) > self.get_clock().now():
            # second move
            self.joint_angles = [1.0, 0.5, 0.6, 0.0, 0.5]
            if self.stage == 1:
                print(f"stage 1 called")
                self.send_request(1.0)
                self.goal_kinematics_pose = [-1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0]
                self.send_goal_task_space(0.3)
                self.stage += 1
        elif self.prev_time + Duration(seconds=6) > self.get_clock().now():
            # second move
            self.joint_angles = [0.2, 0.0, 0.6, 0.0, 1.5]
            if self.stage == 2:
                print(f"stage 2 called")
                self.send_request(0.5)
                self.goal_kinematics_pose = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]
                self.send_goal_task_space(0.3)
                self.stage += 1
        elif self.prev_time + Duration(seconds=7) > self.get_clock().now():
            # second move
            self.joint_angles = [0.4, 0.4, 0.2, 0.5, 1.0]
            if self.stage == 3:
                print(f"stage 3 called")
                self.send_request(0.7)
                self.stage += 1

        elif self.prev_time + Duration(seconds=8) > self.get_clock().now():
            # second move
            self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
            if self.stage == 4:
                print(f"stage 4 called")
                self.send_request(1.0)
                self.stage += 1
        elif self.prev_time + Duration(seconds=10) > self.get_clock().now():
            self.prev_time = self.get_clock().now()
            self.stage = 0

def main():
    rclpy.init()
    node = Patrol_manipulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()
