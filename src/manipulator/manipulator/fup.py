import rclpy
from rclpy.node import Node
from math import atan2,cos,sin
from open_manipulator_msgs.srv import ManiPose
from std_msgs.msg import Float64MultiArray
from sympy import *
from scipy.optimize import fsolve
import time

class InvKine(Node):
    def __init__(self):
        super().__init__('Inv_kine')
        self.srv = self.create_service(ManiPose,'calc_inverse_kin',self.calc_inv_kin_callback)
        # self.pub = self.create_publisher(Float64MultiArray,'move_arm',10)
        self.x_val = 0.0
        self.y_val = 0.0
        self.z_val = 0.0
        self.q1 = 0.0


    def equations(self,vars):
        q2, q3, q4 = vars
        q1 = self.q1
        eq1 = 128*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + (1407*cos(q3)*sin(q4 + pi/2)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/5 + (1407*cos(q4 + pi/2)*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))/5 - self.x_val
        eq2 = 128*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + (1407*cos(q3)*sin(q4 + pi/2)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/5 + (1407*cos(q4 + pi/2)*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))/5 - self.y_val
        eq3 = 128*cos(q3) + (1407*cos(q3)*cos(q4 + pi/2))/5 - (1407*sin(q3)*sin(q4 + pi/2))/5 + 48163/500 - self.z_val
        return [eq1, eq2, eq3]

    def calc_inv_kin_callback(self,request,response):
        self.q1 = atan2(request.position[1],request.position[0])
        self.x_val = request.position[0]
        self.y_val = request.position[1]
        self.z_val = request.position[2]
        print("Processing start ---------------")
        
        RHS = [0.2,0.3,0]
        res = fsolve(self.equations,RHS)

        print("Result = ",res)
        self.get_logger().info(f"Result of Inverse Kinematics are theta1: {self.q1} theta2: {res[0]} theta3: {res[1]} theta4: {res[2]}")
        # positions = Float64MultiArray
        # positions.data = [self.q1,res[0],res[1],res[2]]
        # self.pub.publish(positions)
        response.result = True
        return response
    

def main():
    rclpy.init()

    inv_kine = InvKine()
    rclpy.spin(inv_kine)
    rclpy.shutdown()

if __name__ == "__main__":
    main()