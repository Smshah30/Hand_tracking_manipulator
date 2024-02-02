import rclpy
from rclpy.node import Node
from open_manipulator_msgs.msg import FingerPose
from open_manipulator_msgs.srv import ManiPose
from geometry_msgs.msg import Pose
import time



class Connector(Node):

    def __init__(self):
        super().__init__('connector_arm_hand')

        self.cli_joint = self.create_client(ManiPose, 'calc_inverse_kin')
        self.pose_sub = self.create_subscription(Pose,'end_effector_pose',self.end_pose_callback,10)
        self.hand_pose_sub = self.create_subscription(FingerPose,'location_arm',self.hand_pose_callback,10)
        
        self.image_pose = [0.5,0.5]
        self.gripper = 0
        self.old_gripper = 0
        self.arm_pose = {'position':[245.0,-0.3,128.0],'orientation':[-0.70698,0.013,0.0130,0.70698],'gripper': -1.0}

        while not self.cli_joint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ManiPose.Request()
        self.send_request_joint()



    def end_pose_callback(self,msg:Pose):
        self.arm_pose['position'] = [msg.position.x,msg.position.y,msg.position.z]
        self.arm_pose['orientation'] = [msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z]

    def hand_pose_callback(self,msg:FingerPose):
        centr_fing,tip_mdf,centr_plm = (msg.centre_finger,msg.tip_middlef,msg.centre_palm)     # centre of finger, tip of Middle finger, Centre of Palm (9,12,0 index mediapipe)

        width,height = (1280,720)

        movement_y = ((centr_fing[0] - self.image_pose[0]) * width)/42   # Mapped movement of 1280 to max real movement of 30 cms. (1280/30) = 42.667
        movement_z = (-(centr_fing[1] - self.image_pose[1]) * height)/24 # Mapped movement of 720 to max real movement of 30 cms. (720/30) = 24
        movement_x = 0                  # Ignoring for now
        factor = 15
        diff = centr_plm[1] - tip_mdf[1]   # diff : max = 0.39, min = 0.08 scale this between -0.009 to 0.009 
        self.gripper = ((diff - 0.08) * (0.018/0.31)) - 0.009    # scaling to -0.009 to 0.009

        if abs(self.old_gripper-self.gripper) * 1000 >= 3:
            self.arm_pose['gripper'] = self.gripper
            self.old_gripper = self.gripper
            result = self.send_request_joint()
            time.sleep(1)

        else:
            self.arm_pose['gripper'] = -1.0

        if abs(movement_z) > 1.2 or abs(movement_y) > 1.2:

            print("Movement ----------------------------",movement_y,movement_z)
            temp = self.arm_pose['position']

            if abs(movement_y) < 1.2 :
                movement_y = 0
            elif abs(movement_z) <1.2:
                movement_z = 0.2

            if (temp[2] + (factor * movement_z)) < 20:
                self.arm_pose['position'] = [temp[0] + movement_x ,temp[1] ,20.0]
            else:
                # self.arm_pose['position'] = [temp[0] + movement_x ,temp[1] ,temp[2] + (factor * movement_z)]
                # self.arm_pose['position'] = [temp[0] + movement_x ,temp[1] + (factor * movement_y) ,temp[2] ]
                self.arm_pose['position'] = [temp[0] + movement_x ,temp[1] + (factor * movement_y) ,temp[2] + (factor * movement_z)]



            self.arm_pose['gripper'] = -1.0
            self.image_pose = [centr_fing[0],centr_fing[1]]


            result = self.send_request_joint()
            time.sleep(5)


            self.get_logger().info(f"Position sent to move : {self.arm_pose} \n result = {result}")
        else:
            print("Movement too less")


    def send_request_joint(self):
        self.req.position = self.arm_pose['position'] 
        self.req.orientation = self.arm_pose['orientation'] 
        self.req.gripper = self.arm_pose['gripper'] 
        self.future = self.cli_joint.call_async(self.req)
        # rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()




def main():
    rclpy.init()
    connector = Connector()
    rclpy.spin(connector)
    connector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()