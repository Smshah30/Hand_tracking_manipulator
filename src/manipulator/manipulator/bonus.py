import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import ManiPose
from open_manipulator_msgs.msg import JointPosition
import time

class Basic_control(Node):
    def __init__(self):
        super().__init__('basic_control')
        self.cli_joint = self.create_client(ManiPose, 'calc_inverse_kin')

        while not self.cli_joint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ManiPose.Request()


        

    def send_request_joint(self,pos,quat,grip):
        self.req.position = pos
        self.req.orientation = quat
        self.req.gripper = grip
        self.future = self.cli_joint.call_async(self.req)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()
    
    


def main():
    rclpy.init()
    basic_controller = Basic_control()

    pos = [286.08119805004014, -0.4388434180097812, 211.6326301461998]
    quat = [-0.7068938138374652, 0.01735326938426857, 0.01626888783289885, 0.7069196017148488]
    grip = -1.0
    response = basic_controller.send_request_joint(pos=pos,quat=quat,grip=grip)
    time.sleep(5.0)

    grip = 0.009
    response = basic_controller.send_request_joint(pos=pos,quat=quat,grip=grip)
    time.sleep(3.0)

    pos = [146.6,-0.2248,90.0]
    quat = [-0.4911,0.5087,0.5079,0.4918]
    grip = -1.0

    response = basic_controller.send_request_joint(pos=pos,quat=quat,grip=grip)
    time.sleep(5.0)

    grip = 0.009
    response = basic_controller.send_request_joint(pos=pos,quat=quat,grip=grip)
    time.sleep(3.0)

    pos = [146.6,-0.2248,30.0]
    quat = [-0.4911,0.5087,0.5079,0.4918]
    grip = -1.0


    response = basic_controller.send_request_joint(pos=pos,quat=quat,grip=grip)
    time.sleep(5.0)

    grip = -0.005
    response = basic_controller.send_request_joint(pos=pos,quat=quat,grip=grip)
    time.sleep(3.0)

    pos = [146.6,-0.2248,90.0]
    quat = [-0.4911,0.5087,0.5079,0.4918]
    grip = -1.0

    response = basic_controller.send_request_joint(pos=pos,quat=quat,grip=grip)


    basic_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
