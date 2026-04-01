#!/usr/bin/env python3

from builtin_interfaces.msg import Duration, Time
from gazebo_msgs.srv import ApplyJointEffort
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import JointState


class JointEffortController(Node):
    """Simple proportional effort controller for a single Gazebo joint."""

    def __init__(self) -> None:
        super().__init__('joint_effort_controller')

        self.declare_parameter('joint_name', 'joint3')
        self.declare_parameter('target_position', 0.0)
        self.declare_parameter('Kp', 5.0)
        self.declare_parameter('effort_bias', 0.0)
        self.declare_parameter('max_effort', 100.0)
        self.declare_parameter('effort_duration_sec', 1.0)
        self.declare_parameter('motion_epsilon', 1e-5)
        self.declare_parameter('auto_reverse_on_stall', True)
        self.declare_parameter('effort_service', '/apply_joint_effort')

        self.joint_name = self.get_parameter('joint_name').value
        self.target_position = float(self.get_parameter('target_position').value)
        self.kp = float(self.get_parameter('Kp').value)
        self.effort_bias = float(self.get_parameter('effort_bias').value)
        self.max_effort = abs(float(self.get_parameter('max_effort').value))
        self.effort_duration_sec = float(self.get_parameter('effort_duration_sec').value)
        self.motion_epsilon = float(self.get_parameter('motion_epsilon').value)
        self.auto_reverse_on_stall = bool(self.get_parameter('auto_reverse_on_stall').value)
        self.effort_service = self.get_parameter('effort_service').value

        self.current_position = None
        self.pending_future = None
        self.last_sent_position = None
        self.last_sent_effort = 0.0
        self.reverse_attempted = False
        self.motion_check_timer = None

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.client = self.create_client(ApplyJointEffort, self.effort_service)

        self.get_logger().info(
            f'Waiting for {self.effort_service} service for joint "{self.joint_name}"...'
        )
        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f'{self.effort_service} not available yet, retrying...'
            )

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f'Started controller with joint_name={self.joint_name}, '
            f'target_position={self.target_position:.3f}, Kp={self.kp:.3f}, '
            f'effort_bias={self.effort_bias:.3f}, max_effort={self.max_effort:.3f}, '
            f'effort_duration_sec={self.effort_duration_sec:.3f}'
        )

    def joint_state_callback(self, msg: JointState) -> None:
        joint_index = self._find_joint_index(msg.name)
        if joint_index is None:
            return

        if joint_index < len(msg.position):
            self.current_position = msg.position[joint_index]

    def _find_joint_index(self, joint_names) -> int | None:
        # Support exact names and scoped names like model::joint or /ns/joint.
        for idx, name in enumerate(joint_names):
            if name == self.joint_name:
                return idx

        suffixes = (f'::{self.joint_name}', f'/{self.joint_name}')
        for idx, name in enumerate(joint_names):
            if name.endswith(suffixes):
                return idx

        return None

    def control_loop(self) -> None:
        if self.current_position is None:
            self.get_logger().warn(
                f'No position received yet for joint "{self.joint_name}" from /joint_states.'
            )
            return

        if self.pending_future is not None and not self.pending_future.done():
            return

        error = self.target_position - self.current_position
        effort = self.effort_bias + (self.kp * error)
        effort = max(-self.max_effort, min(self.max_effort, effort))

        request = ApplyJointEffort.Request()
        request.joint_name = self.joint_name
        request.effort = effort
        request.start_time = Time()
        sec = int(self.effort_duration_sec)
        nanosec = int((self.effort_duration_sec - sec) * 1e9)
        request.duration = Duration(sec=sec, nanosec=nanosec)

        self.last_sent_position = self.current_position
        self.last_sent_effort = effort
        self.pending_future = self.client.call_async(request)
        self.pending_future.add_done_callback(self._handle_apply_effort_response)

        self.get_logger().info(
            f'joint={self.joint_name}, current_position={self.current_position:.4f}, '
            f'target={self.target_position:.4f}, error={error:.4f}, effort={effort:.4f}'
        )

    def _handle_apply_effort_response(self, future) -> None:
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Effort command accepted by Gazebo.')
                # Check shortly after success whether position actually changed.
                self.motion_check_timer = self.create_timer(0.25, self._check_motion_once)
            else:
                self.get_logger().warn(
                    f'Gazebo rejected effort command: {response.status_message}'
                )
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'Failed to call apply_joint_effort service: {exc}')

    def _check_motion_once(self) -> None:
        if self.motion_check_timer is not None:
            self.motion_check_timer.cancel()
            self.motion_check_timer = None

        if self.current_position is None or self.last_sent_position is None:
            return

        moved = abs(self.current_position - self.last_sent_position)
        if moved >= self.motion_epsilon:
            self.reverse_attempted = False
            return

        if not self.auto_reverse_on_stall or self.reverse_attempted:
            self.get_logger().warn(
                f'No motion detected after successful effort call (delta={moved:.6g}).'
            )
            return

        reverse_effort = -self.last_sent_effort
        request = ApplyJointEffort.Request()
        request.joint_name = self.joint_name
        request.effort = reverse_effort
        request.start_time = Time()
        sec = int(self.effort_duration_sec)
        nanosec = int((self.effort_duration_sec - sec) * 1e9)
        request.duration = Duration(sec=sec, nanosec=nanosec)

        self.reverse_attempted = True
        self.last_sent_position = self.current_position
        self.last_sent_effort = reverse_effort
        self.pending_future = self.client.call_async(request)
        self.pending_future.add_done_callback(self._handle_apply_effort_response)
        self.get_logger().warn(
            'Service succeeded but joint did not move; trying reverse effort '
            f'{reverse_effort:.4f} once.'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointEffortController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()