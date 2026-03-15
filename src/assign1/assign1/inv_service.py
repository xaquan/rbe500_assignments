from geometry_msgs.msg import Transform
from .scara_kinematic_model import ScaraKinematicModel
from .converter_helper import ConverterHelper    

import rclpy
from rclpy.node import Node


class InverseKinematicsService(Node):

    def __init__(self):
        super().__init__('inverse_kinematics_service')
        self.srv = self.create_service(Transform, 'pose_to_joint_angles', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        print("Received request for inverse kinematics service")
        print(f"Requested pose:\n{request}")

        return response


def main():
    rclpy.init()

    inverse_kinematics_service = InverseKinematicsService()

    rclpy.spin(inverse_kinematics_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()