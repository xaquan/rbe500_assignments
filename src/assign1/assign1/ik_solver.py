import sys
from assign1_interfaces.srv import PoseToJointAngles
from .scara_kinematic_model import ScaraKinematicModel
from geometry_msgs.msg import Pose

import rclpy
from rclpy.node import Node

class InverseKinematicsClient(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_client')
        self.cli = self.create_client(PoseToJointAngles, 'pose_to_joint_angles')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = PoseToJointAngles.Request()

    def send_request(self, ee_pose):

        self.req.ee_pose = ee_pose
        self.future = self.cli.call_async(self.req)
        return self.future


def main():
    rclpy.init()

    inverse_kinematics_client = InverseKinematicsClient()

    ee_pose = Pose()
    ee_pose.position.x = float(sys.argv[1])
    ee_pose.position.y = float(sys.argv[2])
    ee_pose.position.z = float(sys.argv[3])
    ee_pose.orientation.x = float(sys.argv[4])
    ee_pose.orientation.y = float(sys.argv[5])
    ee_pose.orientation.z = float(sys.argv[6])
    ee_pose.orientation.w = float(sys.argv[7])


    future = inverse_kinematics_client.send_request(ee_pose)
    rclpy.spin_until_future_complete(inverse_kinematics_client, future)
    reponse = future.result()
    if reponse is not None:
        print(f"Received joint angles:\n{reponse.joint_angles}")
    else:
        print("Service call failed")

    rclpy.shutdown()


if __name__ == '__main__':
    main()