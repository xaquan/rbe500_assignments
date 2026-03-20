import sys
from assign1_interfaces.srv import PoseToJointAngles
from .scara_kinematic_model import ScaraKinematicModel
from geometry_msgs.msg import Pose    
from rosidl_runtime_py.set_message import set_message_fields
import yaml


import rclpy
from rclpy.node import Node

class InverseKinematicsClient(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_client')
        self.cli = self.create_client(PoseToJointAngles, 'pose_to_joint_angles')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = PoseToJointAngles.Request()

    def send_request(self, msg):

        self.req.ee_pose = msg
        self.future = self.cli.call_async(self.req)
        return self.future

def main():
    rclpy.init()

    inverse_kinematics_client = InverseKinematicsClient()

    data = yaml.safe_load(sys.argv[1])
    msg =  Pose()

    set_message_fields(msg, data)

    future = inverse_kinematics_client.send_request(msg)
    rclpy.spin_until_future_complete(inverse_kinematics_client, future)
    reponse = future.result()
    if reponse is not None:
        print(f"Calculated joint angles:\n{reponse.joint_angles}")
    else:
        print("Service call failed")

    rclpy.shutdown()


if __name__ == '__main__':
    main()