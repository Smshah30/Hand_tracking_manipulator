import mediapipe as mp
import cv2 
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from open_manipulator_msgs.msg import FingerPose



# def calculate_angle(a,b,c):
#     a = np.array(a) # First
#     b = np.array(b) # Mid
#     c = np.array(c) # End
    
#     # radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])  # XY plane angle
#     radians = np.arctan2(c[2]-b[2], c[0]-b[0]) - np.arctan2(a[2]-b[2], a[0]-b[0])  # XZ plane angle

#     angle = np.abs(radians*180.0/np.pi)
    
#     if angle >180.0:
#         angle = 360-angle
        
#     return angle 

global arm_pose

arm_pose = None

class Detection():

    def __init__(self) -> None:
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.mp_hand = mp.solutions.holistic

    def run(self):
        # with self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        global arm_pose
        with self.mp_hand.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
            while self.cap.isOpened():
                ret, frame = self.cap.read()

                # Recolor image to RGB
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image.flags.writeable = False

                # Make detection
                results = holistic.process(image)
                # results_hand = 
                try:

                    rhand_landmarks = results.right_hand_landmarks.landmark

                    self.centr = rhand_landmarks[9]
                    self.tip_mfing = rhand_landmarks[12]
                    self.palm = rhand_landmarks[0]

                    # print("Centre Finger: ",self.centr)
                    # print("tip_middle finger: ",self.tip_mfing)
                    # print("Palm: ",self.palm)

                    arm_pose = [self.centr,self.tip_mfing,self.palm]



                except:
                    pass
                # Recolor back to BGR
                # image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                # try:
                #     landmarks = results.pose_landmarks.landmark
                #     left_shld = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value]
                #     right_shld = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
                #     left_elb = landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW.value]

                #     # ang = calculate_angle([right_shld.x,right_shld.y,right_shld.z],[left_shld.x,left_shld.y,left_shld.z],[left_elb.x,left_elb.y,left_elb.z])

                #     # cv2.putText(image,str(ang),
                #                 # (20,20),
                #                 # cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),2,cv2.LINE_AA)

                # except:
                #     pass



                
                # Render detections
                self.mp_drawing.draw_landmarks(image, results.right_hand_landmarks, self.mp_hand.HAND_CONNECTIONS,
                                        self.mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2), 
                                        self.mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2) 
                                         )               

                cv2.imshow('Mediapipe Feed', image)

                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

            self.cap.release()
            cv2.destroyAllWindows()


class RosSup(Node):
    def __init__(self) -> None:
        super().__init__('ros_sup')
        
        self.pub = self.create_publisher(FingerPose,'location_arm',10)
        self.old_arm_pose = None
    
    def run(self):
        global arm_pose
        while rclpy.ok():
            # print("ROS running --------------")
            if self.old_arm_pose == arm_pose or arm_pose == None:
                continue
            else:
                print("This is the pose i got",arm_pose)
                self.old_arm_pose = arm_pose
                msg = FingerPose()
                # msg.data = arm_pose
                msg.centre_finger = [arm_pose[0].x,arm_pose[0].y,arm_pose[0].z]
                msg.tip_middlef = [arm_pose[1].x,arm_pose[1].y,arm_pose[1].z]
                msg.centre_palm = [arm_pose[2].x,arm_pose[2].y,arm_pose[2].z]
                self.pub.publish(msg=msg)

                print("Y gap variation ====================== ",(arm_pose[1].y) - (arm_pose[2].y))
                pass

def main():
    rclpy.init()
    detection = Detection()
    ros_sup = RosSup()
    # executor = rclpy.executors.SingleThreadExecutor()
    # executor.add_node(ros_sup)
    executor_thread = threading.Thread(target=ros_sup.run,daemon=True)
    executor_thread.start()
    detection.run()

    rclpy.shutdown()
    executor_thread.join()

main()