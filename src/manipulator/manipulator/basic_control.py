import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.msg import JointPosition


class Basic_control(Node):
    def __init__(self):
        super().__init__('basic_control')
        self.cli = self.create_client(SetJointPosition, 'goal_joint_space_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetJointPosition.Request()

    def send_request(self):
        self.req.planning_group=''
        self.req.joint_position.joint_name=["joint1","joint2","joint3","joint4","gripper"]
        self.req.joint_position.position= [0.0,0.0,0.0,0.0,0.0]                  #[-0.0015,0.55513,0.4805,0.5014,-0.009]
        # self.req.joint_position.position=[0.5,0.0,0.0,1.4,-0.009]
        self.req.path_time = 4.0
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()
    


def main():
    rclpy.init()
    basic_controller = Basic_control()
    response = basic_controller.send_request()
    basic_controller.get_logger().info(f'Connected to Arm: {response.is_planned}')

    basic_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
