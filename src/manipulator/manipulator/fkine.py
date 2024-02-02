import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
from sensor_msgs.msg import JointState
import math 
from math import cos,sin
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

class fkine(Node):
    def __init__(self):
        super().__init__('fkine')
        self.pos_subs = self.create_subscription(JointState,'joint_states',self.joint_pos_callback,10)
        self.pos_publisher = self.create_publisher(Pose,"end_effector_pose",10)
        # self.DH = [[],                                  # a      
        #            [],                                  #alpha
        #            [],                                  #theta
        #            []]                                  #d

    def joint_pos_callback(self,msg):
        joint_pose = msg.position
        self.forwardTransfer(q1=joint_pose[0],q2=joint_pose[1],q3=joint_pose[2],q4=joint_pose[3])


    def forwardTransfer(self,q1,q2,q3,q4):
        pi = math.pi


        T =[
 
            [cos(q4)*(cos(q2 - 1.3850)*cos(q3 + 1.3850)*cos(q1) - sin(q2 - 1.3850)*sin(q3 + 1.3850)*cos(q1)) - sin(q4)*(cos(q2 - 1.3850)*sin(q3 + 1.3850)*cos(q1) + cos(q3 + 1.3850)*sin(q2 - 1.3850)*cos(q1)), - cos(q4)*(cos(q2 - 1.3850)*sin(q3 + 1.3850)*cos(q1) + cos(q3 + 1.3850)*sin(q2 - 1.3850)*cos(q1)) - sin(q4)*(cos(q2 - 1.3850)*cos(q3 + 1.3850)*cos(q1) - sin(q2 - 1.3850)*sin(q3 + 1.3850)*cos(q1)), -sin(q1), 133.4000*cos(q4)*(cos(q2 - 1.3850)*cos(q3 + 1.3850)*cos(q1) - sin(q2 - 1.3850)*sin(q3 + 1.3850)*cos(q1)) - 133.4000*sin(q4)*(cos(q2 - 1.3850)*sin(q3 + 1.3850)*cos(q1) + cos(q3 + 1.3850)*sin(q2 - 1.3850)*cos(q1)) + 130.2300*cos(q2 - 1.3850)*cos(q1) + 124*cos(q2 - 1.3850)*cos(q3 + 1.3850)*cos(q1) - 124*sin(q2 - 1.3850)*sin(q3 + 1.3850)*cos(q1)],
            [cos(q4)*(cos(q2 - 1.3850)*cos(q3 + 1.3850)*sin(q1) - sin(q2 - 1.3850)*sin(q3 + 1.3850)*sin(q1)) - sin(q4)*(cos(q2 - 1.3850)*sin(q3 + 1.3850)*sin(q1) + cos(q3 + 1.3850)*sin(q2 - 1.3850)*sin(q1)), - cos(q4)*(cos(q2 - 1.3850)*sin(q3 + 1.3850)*sin(q1) + cos(q3 + 1.3850)*sin(q2 - 1.3850)*sin(q1)) - sin(q4)*(cos(q2 - 1.3850)*cos(q3 + 1.3850)*sin(q1) - sin(q2 - 1.3850)*sin(q3 + 1.3850)*sin(q1)),  cos(q1), 133.4000*cos(q4)*(cos(q2 - 1.3850)*cos(q3 + 1.3850)*sin(q1) - sin(q2 - 1.3850)*sin(q3 + 1.3850)*sin(q1)) - 133.4000*sin(q4)*(cos(q2 - 1.3850)*sin(q3 + 1.3850)*sin(q1) + cos(q3 + 1.3850)*sin(q2 - 1.3850)*sin(q1)) + 130.2300*cos(q2 - 1.3850)*sin(q1) + 124*cos(q2 - 1.3850)*cos(q3 + 1.3850)*sin(q1) - 124*sin(q2 - 1.3850)*sin(q3 + 1.3850)*sin(q1)],
            [                              - cos(q4)*(cos(q2 - 1.3850)*sin(q3 + 1.3850) + cos(q3 + 1.3850)*sin(q2 - 1.3850)) - sin(q4)*(cos(q2 - 1.3850)*cos(q3 + 1.3850) - sin(q2 - 1.3850)*sin(q3 + 1.3850)),                                   sin(q4)*(cos(q2 - 1.3850)*sin(q3 + 1.3850) + cos(q3 + 1.3850)*sin(q2 - 1.3850)) - cos(q4)*(cos(q2 - 1.3850)*cos(q3 + 1.3850) - sin(q2 - 1.3850)*sin(q3 + 1.3850)),        0,                                               96.3260 - 124*cos(q2 - 1.3850)*sin(q3 + 1.3850) - 124*cos(q3 + 1.3850)*sin(q2 - 1.3850) - 133.4000*cos(q4)*(cos(q2 - 1.3850)*sin(q3 + 1.3850) + cos(q3 + 1.3850)*sin(q2 - 1.3850)) - 133.4000*sin(q4)*(cos(q2 - 1.3850)*cos(q3 + 1.3850) - sin(q2 - 1.3850)*sin(q3 + 1.3850)) - 130.2300*sin(q2 - 1.3850)],
            [                                                                                                                                                                                                0,                                                                                                                                                                                                   0,        0,                                                                                                                                                                                                                                                                                                                                                       1]
        ]
        
        pos = Pose()
        pos.position.x = T[0][3]
        pos.position.y = T[1][3]
        pos.position.z = T[2][3]

        if T[2][0] == 1 or T[2][0] == -1:
            alpha = pi
            psi = math.atan2(T[0][1],-T[1][1])
            beta = pi/2
        else:
            alpha = math.atan2(T[2][1],T[2][2])
            beta = math.atan2(-T[2][0],math.sqrt(math.pow(T[2][1],2)+math.pow(T[2][2],2)))
            psi = math.atan2(T[1][0],T[0][0])

        rot = Rotation.from_euler('xyz',[alpha,beta,psi],degrees=False)
        print("Euler Angles : ",[alpha,beta,psi])
        rot = rot.as_quat()

        pos.orientation.x = rot[1]
        pos.orientation.y = rot[2]
        pos.orientation.z = rot[3]
        pos.orientation.w = rot[0]

        self.pos_publisher.publish(pos)
        print(pos)

def main():
    rclpy.init()

    forwardkin = fkine()
    rclpy.spin(forwardkin)

    forwardkin.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

