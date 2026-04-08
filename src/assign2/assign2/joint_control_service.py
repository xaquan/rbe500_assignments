from datetime import datetime
import math
import os
import queue
import threading
import rclpy
from rclpy.node import Clock, Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from assignment_interfaces.srv import SetJointEffort


class JointControlService(Node):
    def __init__(self):
        super().__init__('joint_control_service')

        # self.declare_parameter('command_topic', '/model/scara_robot/joint/joint3/cmd_force')
        # self.declare_parameter('joint_states_topic', '/joint_states')
        # self.declare_parameter('service_name', 'set_joint_effort')
        self.declare_parameter('joint_name', 'joint3')
        self.declare_parameter('kd', 17)
        self.declare_parameter('kp', 114)
        self.declare_parameter('dt', 0.02)  # 20ms control loop

        # command_topic = str(self.get_parameter('command_topic').value)
        joint_name = str(self.get_parameter('joint_name').value)
        command_topic = f'/model/scara_robot/joint/{joint_name}/cmd_force'
        joint_states_topic = "/joint_states"
        service_name = "set_joint_position"
        
        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.dt = float(self.get_parameter('dt').value)

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

        self._effort_pub = self.create_publisher(Float64, command_topic, 10)
        self._joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_states_callback,
            10,
        )
        self._srv = self.create_service(SetJointEffort, service_name, self._handle_request)
        # self._control_timer = self.create_timer(self.dt, self._control_loop)
        
        self.get_logger().info(
            'Joint control service ready: '
            f"service='{service_name}', topic='{command_topic}', "
            f"joint_states='{joint_states_topic}', tracked_joint='{self.tracked_joint_name}'"
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

    

    def _joint_states_callback(self, msg: JointState) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.dt = stamp - self.last_joint_state_stamp if self.last_joint_state_stamp is not None else 0.0
        self.last_joint_state_stamp = stamp

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

        

    # def _clear_goal(self):
    #     self.target_joint_position = None
    #     self._pre_error = 0.0
    #     self.active_goal = False
    #     self.get_logger().info(f'Cleared target for joint "{self.tracked_joint_name}"') 
    
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
    

    def _apply_effort(self):
                       
        error = self.target_joint_position - self.joint_current_position

        # if abs(error) <= self.tolerance:
        #     self.get_logger().info(
        #         f'Goal reached for "{self.tracked_joint_name}": '
        #         f'current={self.joint_current_position:.4f}, '
        #         f'target={self.target_joint_position:.4f}, error={error:.4f}'
        #     )
        #     return True

        m = 10
        b = 0.5
        xi = 1
        ts = 0.5
        mg = m * -9.81  # Estimate gravity disturbance based on current joint position
        wn = 5.3/(ts*xi)
        kp = wn**2 * m
        kd = 2*xi*wn*m - b

        # Log the calculated PD parameters
        self.get_logger().info(
            f'Calculated PD parameters for joint "{self.tracked_joint_name}": '
            f'ts={ts:.4f}, xi={xi:.4f}, kp={kp:.4f}, kd={kd:.4f}, mg={mg:.4f}, wn={wn:.4f}'
        )
        pd_output = self._PD_Controller(error, kp, kd, self.dt)
        effort = pd_output + mg  # Compensate for estimated gravity disturbance
        
        self.get_logger().info(
            f'Calculated effort={pd_output:.4f} to apply to joint "{self.tracked_joint_name}" '
            f'Publishing effort={effort:.4f} to joint "{self.tracked_joint_name}" '
            f'(current={self.joint_current_position:.4f}, '
            f'target={self.target_joint_position:.4f}, error={error:.4f})'
        )

        self._publish_cmd_force(effort)
        self._save_position_data(error, effort)  

        return False
    
    # Publish effort command to the joint
    def _publish_cmd_force(self, effort: float):
        msg = Float64()
        msg.data = effort
        self._effort_pub.publish(msg)

    # PD controller
    def _PD_Controller(self, error: float, kp: float, kd: float, dt: float) -> float:
       

        derivative = (error - self._pre_error) / dt
        # effort = self.kp * error + self.kd * derivative
        effort = kp * error + kd * derivative
        self._pre_error = error

        return effort


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
