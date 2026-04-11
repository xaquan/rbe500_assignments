import sys

import rclpy
from rclpy.node import Node

from assignment_interfaces.srv import SetJointPosition


class JointControlClient(Node):
    def __init__(self):
        super().__init__('joint_control_client')
        self.cli = self.create_client(SetJointPosition, 'set_joint_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetJointPosition.Request()

    def send_request(self, joint_name: str, target_position: float):
        self.req.joint_name = joint_name
        self.req.target_position = target_position
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: ros2 run assign2 joint_control_client <joint_name> <target_position>')
        rclpy.shutdown()
        return

    joint_name = sys.argv[1]

    try:
        target_position = float(sys.argv[2])
    except ValueError:
        print(f'Invalid target_position: {sys.argv[2]}')
        rclpy.shutdown()
        return

    joint_control_client = JointControlClient()
    future = joint_control_client.send_request(joint_name, target_position)

    rclpy.spin_until_future_complete(joint_control_client, future)
    response = future.result()

    if response is not None:
        print(
            f'status={response.status}, '
            f'joint_name={response.joint_name}, '
            f'msg="{response.msg}"'
        )
    else:
        print('Service call failed')

    joint_control_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
