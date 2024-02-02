import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetJointPosition
import numpy as np
from math import sin,cos,pi
from numpy.linalg import pinv


class IncUpdate(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("Started the node:"+node_name)
        self.subscribers()        
        self.rate = self.create_rate(1)
        self.delta_q = [0.0,0.0,0.0,0.0,0.0]
        self.samp_time = 0.1
        self.value = False
        # self.create_timer(0.1,self.run)
        self.cli = self.create_client(SetJointPosition, 'goal_joint_space_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetJointPosition.Request()
        self.end_eff_vel = np.array([0.0,1.0,0.0,0.0,0.0,0.0])
        self.J = []
        self.old_jp = [0.0,0.0,0.0,0.0,0.0]
        self.jp = [0.0,0.0,0.0,1.57,-0.009]

        self.create_timer(0.1,self.run)




    def subscribers(self):
        self.pos_subs = self.create_subscription(JointState,'joint_states',self.joint_pos_callback,10)
        self.joint_vel_subs = self.create_subscription(Float64MultiArray,'joint_velocities',self.joint_vel_callback,10)
    
    def joint_vel_callback(self,msg):
        data = msg.data


    def joint_pos_callback(self,msg):
        self.joint_pose = np.array(msg.position)
        self.value = True
        q1=self.joint_pose[0]
        q2=self.joint_pose[1]
        q3=self.joint_pose[2]
        q4=self.joint_pose[3]
        # self.jp = [self.joint_pose[0],self.joint_pose[1],self.joint_pose[2],self.joint_pose[3],0.0]

        self.J = [
            
                    [133.4000*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - 128*cos(q2)*sin(q1) + 133.4000*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + 148*sin(q1)*sin(q2)*sin(q3) - 148*cos(q2)*cos(q3)*sin(q1),                                                                                                                                                                                                                                                                                                  -cos(q1)*(128*sin(q2) + 148*cos(q2)*sin(q3) + 148*cos(q3)*sin(q2) + 133.4000*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + 133.4000*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                                                                                                                                                                                                                                    -cos(q1)*(148*cos(q2)*sin(q3) + 148*cos(q3)*sin(q2) + 133.4000*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + 133.4000*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                                                                                                                                                        -cos(q1)*(133.4000*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + 133.4000*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))],
                    [128*cos(q1)*cos(q2) + 133.4000*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - 133.4000*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + 148*cos(q1)*cos(q2)*cos(q3) - 148*cos(q1)*sin(q2)*sin(q3),                                                                                                                                                                                                                                                                                                  -sin(q1)*(128*sin(q2) + 148*cos(q2)*sin(q3) + 148*cos(q3)*sin(q2) + 133.4000*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + 133.4000*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                                                                                                                                                                                                                                    -sin(q1)*(148*cos(q2)*sin(q3) + 148*cos(q3)*sin(q2) + 133.4000*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + 133.4000*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                                                                                                                                                        -sin(q1)*(133.4000*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + 133.4000*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))],
                    [                                                                                                                                                                                                                            0, sin(q1)*(133.4000*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - 128*cos(q2)*sin(q1) + 133.4000*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + 148*sin(q1)*sin(q2)*sin(q3) - 148*cos(q2)*cos(q3)*sin(q1)) - cos(q1)*(128*cos(q1)*cos(q2) + 133.4000*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - 133.4000*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + 148*cos(q1)*cos(q2)*cos(q3) - 148*cos(q1)*sin(q2)*sin(q3)), sin(q1)*(133.4000*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + 133.4000*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + 148*sin(q1)*sin(q2)*sin(q3) - 148*cos(q2)*cos(q3)*sin(q1)) - cos(q1)*(133.4000*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - 133.4000*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + 148*cos(q1)*cos(q2)*cos(q3) - 148*cos(q1)*sin(q2)*sin(q3)), sin(q1)*(133.4000*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + 133.4000*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - cos(q1)*(133.4000*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - 133.4000*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))],
                    [                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                          -sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                              -sin(q1),                                                                                                                                                                                                                                                                                                      -sin(q1)],
                    [                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                           cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                               cos(q1),                                                                                                                                                                                                                                                                                                       cos(q1)],
                    [                                                                                                                                                                                                                            1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                                                                                             0]
 
        ]
        # self.run()
        
    def run(self):
        if self.value:
            numpy_J = np.array(self.J)
            J_inv = pinv(numpy_J)
            print("SHapes",J_inv.shape)

            x = np.matmul(J_inv,self.end_eff_vel.T).astype('float64')
            self.delta_q = list(x.flatten())
            self.delta_q.extend([0.0])


            A = np.multiply(self.delta_q ,self.samp_time)
            self.q_new = np.add(self.jp, A)
            print("Posted: ",self.q_new.flatten())

            self.req.planning_group = ''
            self.req.joint_position.joint_name = ["joint1","joint2","joint3","joint4","gripper"]
            self.req.joint_position.position = list(self.q_new.flatten())
            self.req.path_time = 1.0
            self.future = self.cli.call_async(self.req)
            self.value = False
            self.old_jp = self.jp
            self.jp = self.q_new


def main(args=None):
    rclpy.init(args=args)
    node = IncUpdate("inc_update")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()