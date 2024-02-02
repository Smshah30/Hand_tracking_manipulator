import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from dynamixel_sdk_custom_interfaces.srv import GetCurrent
from dynamixel_sdk_custom_interfaces.msg import SetCurrent
from open_manipulator_msgs.srv import SetTargetPosition
import threading
import time
import csv

class PD_controller(Node):
   
    def __init__(self):
        super().__init__('pd_controller')
       
        self.Kp = 0.01
        self.Kd = 0.01
        self.prev_val = 0
        self.currPosePublisher = self.create_publisher(SetCurrent, '/set_current', 10)
        self.positionClient = self.create_client(GetCurrent, 'get_current')
        while not self.positionClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("client unavailable")
        self.finalPosition = self.create_service(SetTargetPosition, 'set_target_position', self.setFinalPosition)
        self.control = threading.Thread(target=self.control_loop)
        self.targetPosition = None
        self.targeted = threading.Event()
        self.control.start()

        self.csv = open('position_log.csv', 'w', newline='')
        self.writer = csv.writer(self.csv)
        self.writer.writerow(['Time', 'Target Position', 'Current Position'])

    def setFinalPosition(self, request, response):
        self.targetPosition = request.target_position
        response.success = True
        return response

    def control_loop(self):
        self.error_prev = 0
        self.last_time = self.get_clock().now()
        while rclpy.ok():
            if self.targetPosition is not None:
                self.call_position_service(self.targetPosition)
            time.sleep(0.01)

    def call_position_service(self, targetPosition):
        position_req = GetCurrent.Request()
        position_req.id = 14
        future = self.positionClient.call_async(position_req)
        rclpy.spin_until_future_complete(self, future)
        self.pd_ctrl_cb(future, targetPosition)

    def pd_ctrl_cb(self, future, targetPosition):
        try:

            current_position = future.result().current
            # if current_position < 6000:

            error = targetPosition - current_position
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1000000000
            self.last_time = current_time
            error_dot = (error - self.error_prev) / dt
            self.error_prev = error
            ctrl_cmd = self.Kp * error + self.Kd* error_dot
            print("Control Value --------------------------------------",ctrl_cmd)
            # ctrl_cmd = min(20,max(ctrl_cmd,-70))
            # else:
                # ctrl_cmd = 0
            # if abs(self.prev_val-ctrl_cmd) >50 :
                # ctrl_cmd = (ctrl_cmd-self.prev_val)/2
            current_msg = SetCurrent()
            current_msg.id = 14
            current_msg.current = int(ctrl_cmd)
            self.prev_val = ctrl_cmd
            self.currPosePublisher.publish(current_msg)

            self.writer.writerow([self.get_clock().now().to_msg(), targetPosition, current_position])


            if abs(targetPosition - current_position) <= 100:
                self.get_logger().info(f'Goal position {targetPosition} reached')
        except Exception as e:
            self.get_logger().error(f'Error in pd_ctrl_cb: {str(e)}')
   
    def __del__(self):
        self.csv.close()

def main(args=None):
    rclpy.init()
    controller = PD_controller()
    executor = MultiThreadedExecutor()
    rclpy.spin(controller, executor)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()