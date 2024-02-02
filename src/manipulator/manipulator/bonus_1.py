import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.msg import JointPosition
import time
from std_msgs.msg import Float64MultiArray

class Basic_control(Node):
    def __init__(self):
        super().__init__('basic_control')

        self.subs = self.create_subscription(Float64MultiArray,'move_arm',self.move_arm_callback,10)
        self.cli_joint = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.cli_gripper = self.create_client(SetJointPosition, 'goal_tool_control')
        while not self.cli_joint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetJointPosition.Request()

        while not self.cli_gripper.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_grip = SetJointPosition.Request()

    def move_arm_callback(self,msg):
        intr_arm = msg.data
        if intr_arm[-1] == -1.0:
            response = self.send_request_joint(intr_arm)
            self.get_logger().info(f"Arm Moved to {intr_arm} Result: {response.is_planned}")
        else :
            response = self.send_request_gripper(intr_arm[-1])
            self.get_logger().info(f"Gripper Moved to {intr_arm} Result: {response.is_planned}")



    def send_request_joint(self,arm_arr):
        self.req.planning_group=''
        self.req.joint_position.joint_name=["joint1","joint2","joint3","joint4","gripper"]
        self.req.joint_position.position= arm_arr
        self.req.path_time = 3.0
        self.future = self.cli_joint.call_async(self.req)
        # rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()
    
    def send_request_gripper(self,grip):
        self.req_grip.planning_group=''
        self.req_grip.joint_position.joint_name=["joint1","joint2","joint3","joint4","gripper"]
        self.req_grip.joint_position.position=[0.0,0.0,0.0,0.0,grip]
        self.req_grip.path_time = 5.0
        self.future = self.cli_gripper.call_async(self.req_grip)
        # rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()
    
def main():
    rclpy.init()
    basic_controller = Basic_control()
    rclpy.spin(basic_controller)
    basic_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
