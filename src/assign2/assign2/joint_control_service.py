from datetime import datetime
import math
import os
import queue
import threading
import rclpy
from rclpy.node import Clock, Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from assignment_interfaces.srv import SetJointPosition


class JointControlService(Node):
    def __init__(self):
        super().__init__('joint_control_service')

        # command_topic = '/model/scara_robot/joint/joint3/cmd_force'
        joint_states_topic = "/joint_states"
        service_name = "set_joint_position"
        
        self.kp = 0.0
        self.kd = 0.0
        self.dt = 0.0
        self.gravity_compensation = 0.0

        self.last_joint_state_stamp = None
        self.tracked_joint_name = None
        self.joint_positions = {}
        self.joint_current_position = None
        self.target_joint_position = None
        self._pre_error = 0.0
        # self.disturbance = 10 * 9.81  # Estimate gravity disturbance based on current joint position
        self.active_goal = False
        self.tolerance = 0.002
        self.data_directory = 'joint_data'
        self.filepath = None
        self.clock = Clock()
        self._effort_pub = {}

        self._calculate_pd_parameters_joint3()

        # self._effort_pub['joint3'] = self.create_publisher(Float64, command_topic.replace('{joint_name}', 'joint3'), 10)

        self._joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_states_callback,
            10,
        )
        self._srv = self.create_service(SetJointPosition, service_name, self._handle_request)

        self.get_logger().info(
            'Joint control service ready: '
            f"service='{service_name}', "
            f"joint_states='{joint_states_topic}''"
        )

        self.create_data_file()  # Create new data file for logging position data

        self.log_queue = queue.Queue()

        threading.Thread(target=self.logger_thread, daemon=True).start()

    def logger_thread(self):
        with open(self.filepath, 'a') as f:
            while True:
                data = self.log_queue.get()
                if data is None:
                    break
                f.write(data + '\n')

    # Create publishers for each joint, if self._joint_pubs is empty, create publishers for all joints in self.joint_positions
    def _create_publisher(self,  joint_names):
        if not self._effort_pub:
            for joint_name in joint_names:
                topic = f'/model/scara_robot/joint/{joint_name}/cmd_force'
                self._effort_pub[joint_name] = self.create_publisher(Float64, topic, 10)
                self.get_logger().info(f'Created publisher for joint "{joint_name}" on topic "{topic}"')

    def _joint_states_callback(self, msg: JointState) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.dt = stamp - self.last_joint_state_stamp if self.last_joint_state_stamp is not None else 0.0
        self.last_joint_state_stamp = stamp

        self._create_publisher(msg.name)

        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = float(position)

        if self.tracked_joint_name in self.joint_positions:
            self.joint_current_position = self.joint_positions[self.tracked_joint_name]

            if self.active_goal:
                # self.save_data()                
                self._apply_effort()

    # Publish current position and target position to a file for record analysis
    # format timestamp, current_position, target_position, error, effort
    def _save_position_data(self, error: float, effort: float):
        self.log_queue.put(
            f'{self.clock.now().nanoseconds},'
            f'{self.joint_current_position:.4f},'
            f'{self.target_joint_position:.4f},'
            f'{error:.4f},'
            f'{effort:.4f}'
        )

    # Create new data file with header
    # header: timestamp, current_position, target_position, error, effort
    def create_data_file(self):
        if not os.path.exists(self.data_directory):
            os.makedirs(self.data_directory)
        
        # add system timestamp to filename to avoid overwriting existing files
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        
        self.filepath = f'{self.data_directory}/joint_control_data_{self.tracked_joint_name}_{timestamp}.csv'
        with open(self.filepath, 'w') as f:
            f.write('timestamp,current_position,target_position,error,effort\n')
        self.get_logger().info(f'Created new data file: {self.filepath}')

    def _set_goal(self, joint_name: str, target_position: float):
        self.tracked_joint_name = joint_name
        self.target_joint_position = target_position
        self.joint_current_position = self.joint_positions[joint_name]
        self._pre_error = 0.0
        self.active_goal = True
        self.get_logger().info(
            f'Set new goal for joint "{joint_name}": target_position={target_position:.4f}'
        )

    def _handle_request(self, request, response):
        joint_name = request.joint_name.strip() or self.tracked_joint_name
        target_position = float(request.target_position)

        if not math.isfinite(target_position):
            response.status = False
            response.joint_name = joint_name
            response.msg = 'Invalid target_position value'
            return response

        if joint_name not in self.joint_positions:
            response.status = False
            response.joint_name = joint_name
            response.msg = f'No joint state received yet for joint "{joint_name}"'
            return response
        
        self._set_goal(joint_name, target_position)

        response.status = True
        response.joint_name = joint_name
        response.msg = (
            f'Accepted target for {joint_name}: '
            f'current={self.joint_current_position:.4f}, '
            f'target={self.target_joint_position:.4f}'
        )
        return response
    
    def _calculate_pd_parameters_joint3(self):
        m = 10
        j = m
        b = 1
        xi = 1
        ts = 0.5
        self.gravity_compensation = m * -9.81  # Estimate gravity disturbance based on current joint position
        wn = 5.3/(ts*xi)
        self.kp = wn**2 * j - self.gravity_compensation
        self.kd = 2*xi*wn*j - b

        self.get_logger().info(
            f'Calculated PD parameters for joint "{self.tracked_joint_name}": '
            f'ts={ts:.4f}, xi={xi:.4f}, kp={self.kp:.4f}, kd={self.kd:.4f}, mg={self.gravity_compensation:.4f}, wn={wn:.4f}'
        )

    def _apply_effort(self):
                       
        error = self.target_joint_position - self.joint_current_position

        effort = self._PD_Controller(error, self.kp, self.kd, self.dt, self.gravity_compensation)
        
        self._publish_cmd_force(self.tracked_joint_name, effort)
        self._save_position_data(error, effort)  

        return False
    
    # Publish effort command to the joint
    def _publish_cmd_force(self, joint_name: str, effort: float):
        msg = Float64()
        msg.data = effort
        self._effort_pub[joint_name].publish(msg)

    # PD controller
    def _PD_Controller(self, error: float, kp: float, kd: float, dt: float, disturbance: float = 0.0) -> float:
       
        derivative = (error - self._pre_error) / dt
        # effort = self.kp * error + self.kd * derivative
        effort = kp * error + kd * derivative
        self._pre_error = error

        return effort + disturbance


def main(args=None):
    rclpy.init(args=args)
    node = JointControlService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
