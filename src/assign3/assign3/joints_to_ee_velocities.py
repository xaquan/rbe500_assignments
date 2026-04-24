import numpy as np

import rclpy
from rclpy.node import Clock, Node
from .scara_kinematic_model import ScaraKinematicModel
from .scara_jacobian_model import ScaraJacobianModel
from geometry_msgs.msg import Pose

from assignment_interfaces.srv import JointsToEEVelocities


class JointsToEeVelocitiesService(Node):
    def __init__(self):
        super().__init__('joints_to_ee_velocities_service')

        service_name = "joints_to_ee_velocities_service"
        self._srv = self.create_service(JointsToEEVelocities, service_name, self._handle_request)

        self.get_logger().info(
            'Joints to EE velocities service ready: '
            f"service='{service_name}'"
        )        

    def _handle_request(self, request, response):
        velocities = request.joints_velocities
        self.get_logger().info(f"Received joint velocities: {velocities}")
        response.ee_velocities = self.joints_to_ee_velocities(velocities)
        return response 
    
    def joints_to_ee_velocities(self, joint_velocities):
        # Placeholder for the actual kinematic calculations
        # In a real implementation, this would involve using the robot's kinematic model
        # to compute the end-effector velocities based on the joint velocities.
        positions = [np.pi/3, -np.pi/3, 0.0]
        ee_velocities = ScaraJacobianModel.joints_to_ee_velocities(joint_velocities, positions)
        res = [float(v) for v in ee_velocities]
        return res

def main(args=None):
    rclpy.init(args=args)
    node = JointsToEeVelocitiesService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()