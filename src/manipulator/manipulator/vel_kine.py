import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import GetJointVel, GetEndEffVel
import numpy as np
from math import sin,cos,pi
from sensor_msgs.msg import JointState
from numpy.linalg import pinv
from std_msgs.msg import Float64MultiArray

class VelKine(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("Started the node:"+node_name)
        self.srv_joint = self.create_service(GetJointVel,'get_joint_velocities',self.get_joint_vel_callback)
        self.srv_end_eff = self.create_service(GetEndEffVel,'get_end_eff_velocities',self.get_end_eff_vel_callback)
        self.pos_subs = self.create_subscription(JointState,'joint_states',self.joint_pos_callback,10)
        self.joint_vel_pub = self.create_publisher(Float64MultiArray,'joint_velocities',10)
        self.J = []
        

    def joint_pos_callback(self,msg):
        joint_pose = msg.position
        q1=joint_pose[0]
        q2=joint_pose[1]
        q3=joint_pose[2]
        q4=joint_pose[3]

        self.J = [
            
                    [133.4000*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - 128*cos(q2)*sin(q1) + 133.4000*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + 148*sin(q1)*sin(q2)*sin(q3) - 148*cos(q2)*cos(q3)*sin(q1),                                                                                                                                                                                                                                                                                                  -cos(q1)*(128*sin(q2) + 148*cos(q2)*sin(q3) + 148*cos(q3)*sin(q2) + 133.4000*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + 133.4000*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                                                                                                                                                                                                                                    -cos(q1)*(148*cos(q2)*sin(q3) + 148*cos(q3)*sin(q2) + 133.4000*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + 133.4000*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                                                                                                                                                        -cos(q1)*(133.4000*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + 133.4000*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))],
                    [128*cos(q1)*cos(q2) + 133.4000*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - 133.4000*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + 148*cos(q1)*cos(q2)*cos(q3) - 148*cos(q1)*sin(q2)*sin(q3),                                                                                                                                                                                                                                                                                                  -sin(q1)*(128*sin(q2) + 148*cos(q2)*sin(q3) + 148*cos(q3)*sin(q2) + 133.4000*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + 133.4000*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                                                                                                                                                                                                                                    -sin(q1)*(148*cos(q2)*sin(q3) + 148*cos(q3)*sin(q2) + 133.4000*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + 133.4000*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                                                                                                                                                        -sin(q1)*(133.4000*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + 133.4000*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))],
                    [                                                                                                                                                                                                                            0, sin(q1)*(133.4000*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - 128*cos(q2)*sin(q1) + 133.4000*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + 148*sin(q1)*sin(q2)*sin(q3) - 148*cos(q2)*cos(q3)*sin(q1)) - cos(q1)*(128*cos(q1)*cos(q2) + 133.4000*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - 133.4000*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + 148*cos(q1)*cos(q2)*cos(q3) - 148*cos(q1)*sin(q2)*sin(q3)), sin(q1)*(133.4000*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + 133.4000*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + 148*sin(q1)*sin(q2)*sin(q3) - 148*cos(q2)*cos(q3)*sin(q1)) - cos(q1)*(133.4000*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - 133.4000*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + 148*cos(q1)*cos(q2)*cos(q3) - 148*cos(q1)*sin(q2)*sin(q3)), sin(q1)*(133.4000*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + 133.4000*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - cos(q1)*(133.4000*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - 133.4000*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))],
                    [                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                          -sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                              -sin(q1),                                                                                                                                                                                                                                                                                                      -sin(q1)],
                    [                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                           cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                               cos(q1),                                                                                                                                                                                                                                                                                                       cos(q1)],
                    [                                                                                                                                                                                                                            1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                                                                                             0]
 
        ]
        self.joint_pos = [q1,q2,q3,q4]



    def get_joint_vel_callback(self,request,response):
        end_eff_vel = np.array(request.end_eff_vel)[np.newaxis]
        end_eff_vel = end_eff_vel.T

        if self.J ==[] :
            response.result = False
            return response
        numpy_J = np.array(self.J)
        J_inv = pinv(numpy_J)
        print("SHapes",)
        joint_vels = np.matmul(J_inv,end_eff_vel).astype('float64')


        response.result = True 
        print("Result: ",joint_vels.tolist())
        x = Float64MultiArray()
        y = list(joint_vels.flatten())
        z = y.extend([0.000])
        print("delta Q = ",y)
        # z = [round(i,5) for i in y]
        x.data = y
        self.joint_vel_pub.publish(x)
        return response

    def get_end_eff_vel_callback(self,request,response):
        joint_vel = np.array(request.joint_velocities)[np.newaxis]
        joint_vels = joint_vel.T

        numpy_J = np.array(self.J)

        end_eff_vel = np.matmul(numpy_J,joint_vels)

        response.result = True 
        print("Result: ",end_eff_vel.tolist())
        return response
 


def main(args=None):
    rclpy.init(args=args)
    node = VelKine("vel_kine")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()