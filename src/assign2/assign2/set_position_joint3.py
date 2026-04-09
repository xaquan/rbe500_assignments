import sys
import rclpy
from rclpy.node import Node
from assignment_interfaces.srv import SetJointPosition


class SetPositionJoint3(Node):
    def __init__(self):
        super().__init__('set_position_joint3')
        self.cli = self.create_client(SetJointPosition, 'set_joint_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetJointPosition.Request()

    def send_request(self, target_position: float):
        self.req.joint_name = 'joint3'
        self.req.target_position = target_position
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print('Usage: ros2 run assign2 set_position_joint3 <target_position>')
        rclpy.shutdown()
        return

    try:
        target_position = float(sys.argv[1])
    except ValueError:
        print(f'Invalid target_position: {sys.argv[1]}')
        rclpy.shutdown()
        return

    set_position_joint3 = SetPositionJoint3()
    future = set_position_joint3.send_request(target_position)

    rclpy.spin_until_future_complete(set_position_joint3, future)
    response = future.result()

    if response is not None:
        print(
            f'status={response.status}, '
            f'joint_name={response.joint_name}, '
            f'msg="{response.msg}"'
        )
    else:
        print('Service call failed')

    set_position_joint3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
