import rclpy
from rclpy.node import Node
from math import atan2,sin,cos,sqrt,acos,pow
from open_manipulator_msgs.srv import ManiPose
from scipy.spatial.transform import Rotation as R
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
from sympy import *
from scipy.optimize import fsolve
import numpy as np
from open_manipulator_msgs.srv import SetJointPosition

import time

class invkine(Node):
    def __init__(self):
        super().__init__('invkine')
        self.srv = self.create_service(ManiPose,'calc_inverse_kin',self.calc_inv_kin_callback)
        self.pub = self.create_publisher(Float32MultiArray,'move_arm',10)
        self.x_val = 0.0
        self.y_val = 0.0
        self.z_val = 0.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.q4 = 0.0

        self.R04 = []
        self.O_04 = []
        self.O_03 = []
        self.link2_l = 130.23
        self.link3_l = 124.00

        self.cli_joint = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.cli_gripper = self.create_client(SetJointPosition, 'goal_tool_control')
        while not self.cli_joint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetJointPosition.Request()

        while not self.cli_gripper.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_grip = SetJointPosition.Request()

    def calc_inv_kin_callback(self,request,response):
        
        print("Position : ",request.position)
        print("Orientation: ",request.orientation)
        print("Gripper: ",request.gripper)

        self.q1 = atan2(request.position[1],request.position[0])
        self.x_val = request.position[0]
        self.y_val = request.position[1]
        self.z_val = request.position[2]

        r = R.from_quat(request.orientation)
        self.R04 = np.array(r.as_matrix()) 
        self.O_04 = np.array(request.position).T
        unit_array = np.array([1,0,0]).T
        x_rot = self.R04 @ unit_array
        
        self.O_03 = np.add(self.O_04,np.multiply(-133.4,x_rot))
        x_dis = sqrt((pow(self.O_03[0],2)) + (pow(self.O_03[1],2)))
        z_dis = self.O_03[2] - 96.326
        hyp_dis = sqrt((pow(z_dis,2)) + (pow(x_dis,2)))


        temp_val = (((pow(hyp_dis,2)) - (pow(self.link2_l,2)) - (pow(self.link3_l,2)))/(2 * self.link2_l * self.link3_l))
        # if temp_val > 1:
        #     temp_val = 1
        # elif temp_val < -1:
        #     temp_val = -1
            
        self.q3 = acos(temp_val)

        self.q2 = atan2(z_dis,x_dis) + atan2((self.link3_l * sin(self.q3)),(self.link2_l + self.link3_l * cos(self.q3)))

        self.q2 = 1.3854424 - self.q2
        self.q3 = self.q3 - 1.3854424   # 79.38 degrees to radians

        eulr = euler_from_quaternion(request.orientation)
        self.q4 = eulr[1] - self.q3 - self.q2
        
        self.get_logger().info(f"Result of Inverse Kinematics are theta1: {self.q1} theta2: {self.q2} theta3: {self.q3} theta4: {self.q4}")
        try:
            grip = float(request.gripper)
            a1 = float(self.q1)
            a2 = float(self.q2)
            a3 = float(self.q3)
            a4 = float(self.q4)
            a5 = float(grip)
            arr = [a1,a2,a3,a4,a5]
            self.move_arm_callback(arr)
            response.result = True
        except:
            response.result = False
        return response
    


    def move_arm_callback(self,msg):
        intr_arm = msg
        if intr_arm[-1] == -1.0:
            response = self.send_request_joint(intr_arm)
            self.get_logger().info(f"Arm Moved to {intr_arm} Result: {response}")
        else :
            response = self.send_request_gripper(intr_arm)
            self.get_logger().info(f"Gripper Moved to {intr_arm} Result: {response}")



    def send_request_joint(self,arm_arr):
        self.req.planning_group=''
        self.req.joint_position.joint_name=["joint1","joint2","joint3","joint4","gripper"]
        # self.req.joint_position.position= [arm_arr[0],arm_arr[1],arm_arr[2],arm_arr[3],arm_arr[4]]
        self.req.joint_position.position= arm_arr
        self.req.path_time = 5.0
        self.future = self.cli_joint.call_async(self.req)
        # rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()
    
    def send_request_gripper(self,arm_arr):
        self.req_grip.planning_group=''
        self.req_grip.joint_position.joint_name=["joint1","joint2","joint3","joint4","gripper"]
        self.req_grip.joint_position.position= arm_arr
        self.req_grip.path_time = 1.0
        self.future = self.cli_gripper.call_async(self.req_grip)
        # rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()
    

def main():
    rclpy.init()

    inv_kine = invkine()
    rclpy.spin(inv_kine)
    rclpy.shutdown()

if __name__ == "__main__":
    main()