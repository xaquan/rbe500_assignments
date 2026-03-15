import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from .scara_kinematic_model import ScaraKinematicModel
from .converter_helper import ConverterHelper

class JointStatesSubscriber(Node):
    def __init__(self):
        super().__init__('ee_pose_calculator_pubsub')

        # Subscribe to the /joint_states topic to receive joint angles
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        # Publisher for the end-effector pose
        self.publisher_ = self.create_publisher(Transform, 'scara_ee_pose', 10)

    def joint_states_callback(self, msg):
        if len(msg.position) < 3:
            self.get_logger().error(f"Expected at least 3 joint angles, got {len(msg.position)}")
            return
        self.get_logger().info(f"Received joint angles: {msg.position[0:3]}")
                
        positions = np.array(msg.position[0:3])
        kmodel = ScaraKinematicModel()

        # ee_pose = kmodel.forward_kinematics_scara_robot(positions) # Compute the end-effector pose using the forward kinematics function
        
        # Compute the end-effector pose using the symbolic forward kinematics function with the provided joint angles
        ee_pose = kmodel.forward_kinematics_scara_robot(positions)

        pub_msg = ConverterHelper.transform_matrix_to_transform_msg(ee_pose)  # Publish the end-effector pose as a string message
        self.get_logger().info(f"Publishing end-effector pose:\n{pub_msg}")

           
        self.publisher_.publish(pub_msg)  # Publish the end-effector pose

        


def main(args=None):
    rclpy.init(args=args)
    node = JointStatesSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()