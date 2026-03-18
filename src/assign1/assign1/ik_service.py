from assign1_interfaces.srv import PoseToJointAngles
from .scara_kinematic_model import ScaraKinematicModel

import rclpy
from rclpy.node import Node


class InverseKinematicsService(Node):

    def __init__(self):
        super().__init__('inverse_kinematics_service')
        self.srv = self.create_service(PoseToJointAngles, 'pose_to_joint_angles', self.pose_to_joint_angles_callback)

    def pose_to_joint_angles_callback(self, request, response):

        joint_angles = ScaraKinematicModel.inverse_kinematics_scara_robot(request.ee_pose)  # Compute the inverse kinematics solution for the SCARA robot using the provided pose

        # print(f"Computed joint angles:\n{joint_angles}")

        response.joint_angles = joint_angles.tolist()  # Set the response with the computed joint angles
        return response


def main():
    rclpy.init()

    inverse_kinematics_service = InverseKinematicsService()

    rclpy.spin(inverse_kinematics_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()