import numpy as np

import rclpy
from rclpy.node import Clock, Node
from .models.scara_kinematic_model import ScaraKinematicModel
from .models.scara_jacobian_model import ScaraJacobianModel
from geometry_msgs.msg import Pose

from assignment_interfaces.srv import EEToJointsVelocities


class EeToJointsVelocitiesService(Node):
    def __init__(self):
        super().__init__('ee_to_joints_velocities_service')

        service_name = "ee_to_joints_velocities_service"
        self._srv = self.create_service(EEToJointsVelocities, service_name, self._handle_request)

        self.get_logger().info(
            'EE to Joints velocities service ready: '
            f"service='{service_name}'"
        )        

    def _handle_request(self, request, response):
        velocities = request.ee_velocities
        self.get_logger().info(f"Received EE velocities: {velocities}")
        response.joints_velocities = self.ee_to_joints_velocities(velocities, request.current_positions)
        return response 
    
    def ee_to_joints_velocities(self, ee_velocities, positions=[np.pi/3, -np.pi/3, 0.0]):
        joint_velocities = ScaraJacobianModel.ee_to_joints_velocities(ee_velocities, positions)
        res = [float(v) for v in joint_velocities]
        return res

def main(args=None):
    rclpy.init(args=args)
    node = EeToJointsVelocitiesService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
