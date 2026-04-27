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

        self.declare_parameter('control_mode', 'velocity')
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().string_value

        self.get_logger().info(
            'Initializing Joint Control Service with parameters: '
            f'control_mode="{self.control_mode}"'
        )

        self.robot_configuration = {
            'm1j': 10,
            'm1l': 5,
            'm2j': 5,
            'm2l': 5,
            'm3': 11,
            'r': 0.1,
            'a1': 0.45,
            'a2': 0.35
        }

        # command_topic = '/model/scara_robot/joint/joint3/cmd_force'
        joint_states_topic = "/joint_states"
        service_name = "set_joint_position"

        self.PD_params_joint1 = {
            'kp': 0.0,
            'kd': 0.0,
            'ki': 0.0,
            'd' : 0.0
        }

        self.PD_params_joint2 = {
            'kp': 0.0,
            'kd': 0.0,
            'ki': 0.0,
            'd' : 0.0
        }

        self.PD_params_joint3 = {
            'kp': 0.0,
            'kd': 0.0,
            'ki': 0.0,
            'd' : 0.0
        }

        self.PD_params = [self.PD_params_joint1, self.PD_params_joint2, self.PD_params_joint3]
        
        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.gravity_compensation = 0.0

        self.last_joint_state_stamp = None
        self.tracked_joint_name = None
        self.joint_states_positions = [0.0, 0.0, 0.0]
        self.joint_current_positions = [None, None, None]
        self.target_joint_positions = [0.0, 0.0, 0.0]
        self._pre_errors = [0.0, 0.0, 0.0]
        self._integrals = [0.0, 0.0, 0.0]
        # self.disturbance = 10 * 9.81  # Estimate gravity disturbance based on current joint position
        self.active_goal = False
        self.tolerance = 0.002
        self.data_directory = 'joint_data'
        self.filepath = None
        self.clock = Clock()
        self._effort_pub = {}
        self._velocity_pub = {}
        
        # if self.control_mode == 'force' and self._effort_pub or self.control_mode == 'velocity' and self._velocity_pub:
        #     self.get_logger().info('Publishers already created, skipping publisher creation')
        self._create_joints_cmd_publisher()

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

        # Record position data for analysis
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
    def _create_joints_cmd_publisher(self):
        if self.control_mode == 'force' and not self._effort_pub:
            for joint_name in self.joint_names:
                effort_topic = f'/model/scara_robot/joint/{joint_name}/cmd_force'
                self._effort_pub[joint_name] = self.create_publisher(Float64, effort_topic, 10)
                self.get_logger().info(f'Created effort publisher for joint "{joint_name}" on topic "{effort_topic}"')
        
        if self.control_mode == 'velocity' and not self._velocity_pub:
            for joint_name in self.joint_names:
                vel_topic = f'/model/scara_robot/joint/{joint_name}/cmd_vel'
                self._velocity_pub[joint_name] = self.create_publisher(Float64, vel_topic, 10)
                self.get_logger().info(f'Created velocity publisher for joint "{joint_name}" on topic "{vel_topic}"')

    def _joint_states_callback(self, msg: JointState) -> None: # Update PD parameters for joint1 based on current position of joint2
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.dt = stamp - self.last_joint_state_stamp if self.last_joint_state_stamp is not None else 0.0
        self.last_joint_state_stamp = stamp

        
        if self.control_mode == 'force':
            for i, position in enumerate(msg.position):
                self.joint_states_positions[i] = float(position)
                self.joint_current_positions[i] = float(position)

            # for joint_name in self.joint_names:
            #     self.joint_current_positions[self.joint_names.index(joint_name)] = self.joint_positions[joint_name]   

            self._calculate_force_pd_params(self.joint_states_positions[self.joint_names.index('joint2')]) 
            self._apply_effort()
        elif self.control_mode == 'velocity':
            
            for i, velocity in enumerate(msg.velocity):
                self.joint_states_positions[i] = float(velocity)
                self.joint_current_positions[i] = float(velocity)

            # for joint_name in self.joint_names:
            #     self.joint_current_positions[self.joint_names.index(joint_name)] = self.joint_states_positions[joint_name]    

            self._calculate_velocity_pd_params()
            self._apply_velocity()

    # Publish current position and target position to a file for record analysis
    # format timestamp, current_position, target_position, error, effort
    def _save_position_data(self, error: float, effort: float):
        self.log_queue.put(
            f'{self.clock.now().nanoseconds},'
            f'{self.joint_current_positions[0]:.4f},'
            f'{self.target_joint_positions[0]:.4f},'
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
        # self.tracked_joint_name = joint_name
        i = self.joint_names.index(joint_name)
        self.target_joint_positions[i] = target_position
        self.joint_current_positions[i] = self.joint_states_positions[i]
        self._pre_errors[i] = 0.0
        self.active_goal = True
        self.get_logger().info(
            f'Set new goal for joint "{self.joint_names[i]}": target_position={self.target_joint_positions}, current_position={self.joint_current_positions}'
        )

    def _handle_request(self, request, response):
        joint_name = request.joint_name.strip() or self.tracked_joint_name
        self.tracked_joint_name = joint_name
        target_position = float(request.target_position)

        if not math.isfinite(target_position):
            response.status = False
            response.joint_name = joint_name
            response.msg = 'Invalid target_position value'
            return response
        
        self._set_goal(joint_name, target_position)

        response.status = True
        response.joint_name = joint_name
        response.msg = (
            f'Accepted target for {joint_name}: '
            f'current={self.joint_current_positions[self.joint_names.index(joint_name)]:.4f}, '
            f'target={self.target_joint_positions[self.joint_names.index(joint_name)]:.4f}'
        )
        return response

    def _calculate_velocity_pd_params(self):
        # For simplicity, we use fixed PD parameters for velocity control, but they can also be calculated based on the robot dynamics if needed
        
        self.PD_params[self.joint_names.index('joint1')]['kp'] = 200
        self.PD_params[self.joint_names.index('joint1')]['kd'] = 0
        self.PD_params[self.joint_names.index('joint1')]['ki'] = 10

        self.PD_params[self.joint_names.index('joint2')]['kp'] = 200
        self.PD_params[self.joint_names.index('joint2')]['kd'] = 0
        self.PD_params[self.joint_names.index('joint2')]['ki'] = 70

        self.PD_params[self.joint_names.index('joint3')]['kp'] = 10
        self.PD_params[self.joint_names.index('joint3')]['kd'] = 1
        self.PD_params[self.joint_names.index('joint3')]['ki'] = 0
        self.PD_params[self.joint_names.index('joint3')]['d'] = -11  # Estimate gravity disturbance based on current joint position

    def _calculate_force_pd_params(self, theta2: float):
        self._calculate_pd_parameters_joint1(theta2)

        # For simplicity, we use fixed PD parameters for joint2 and joint3, but they can also be calculated based on the robot dynamics if needed
        if self.PD_params[self.joint_names.index('joint2')]['kp'] == 0.0 or self.PD_params[self.joint_names.index('joint2')]['kd'] == 0.0:
            self._calculate_pd_parameters_joint2()

        if self.PD_params[self.joint_names.index('joint3')]['kp'] == 0.0 or self.PD_params[self.joint_names.index('joint3')]['kd'] == 0.0:
            self._calculate_pd_parameters_joint3()

    def _pd_params_calculate(self, b, xi, ts, j, disturbance=0.0):
        wn = 5.3/(ts*xi)
        kp = wn**2 * j + disturbance
        kd = 2*xi*wn*j - b
        return kp, kd

    def _calculate_pd_parameters_joint1(self, theta2: float):
        m1j = self.robot_configuration['m1j']
        m1l = self.robot_configuration['m1l']
        m2j = self.robot_configuration['m2j']
        m2l = self.robot_configuration['m2l']
        m3 = self.robot_configuration['m3']
        r = self.robot_configuration['r']
        a1 = self.robot_configuration['a1']
        a2 = self.robot_configuration['a2']
        l1 = a1**2 + (a2/2)**2 + 2*a1*(a2/2)*math.cos(theta2)
        l2 = a1**2 + a2**2 + 2*a1*a2*math.cos(theta2)

        j1j = 1/2 * m1j * r**2
        j1l = 1/3 * m1l * r**2
        j2j = m2j * a1**2
        j2l = m2l * l1**2
        j3ee = m3 * l2**2

        j = j1j + j1l + j2j + j2l + j3ee
        pd_params = self._pd_params_calculate(b=1, xi=1, ts=0.5, j=j)
        self.PD_params[self.joint_names.index('joint1')]['kp'] = pd_params[0]
        self.PD_params[self.joint_names.index('joint1')]['kd'] = pd_params[1]

    def _calculate_pd_parameters_joint2(self):
        
        m2j = self.robot_configuration['m2j']
        m2l = self.robot_configuration['m2l']
        m3 = self.robot_configuration['m3']
        r = self.robot_configuration['r']
        a2 = self.robot_configuration['a2']
        j2j = 1/2 * m2j * r**2
        j2l = 1/3 * m2l * a2**2
        j3ee = m3 * a2**2

        j = j2j + j2l + j3ee

        pd_params = self._pd_params_calculate(b=1, xi=1, ts=0.5, j=j)
        self.PD_params[self.joint_names.index('joint2')]['kp'] = pd_params[0]
        self.PD_params[self.joint_names.index('joint2')]['kd'] = pd_params[1]

    def _calculate_pd_parameters_joint3(self):
        j = self.robot_configuration['m3']
        self.gravity_compensation = j * 9.81  # Estimate gravity disturbance based on current joint position
        pd_params = self._pd_params_calculate(b=1, xi=1, ts=0.5, j=j, disturbance=self.gravity_compensation)
        self.PD_params[self.joint_names.index('joint3')]['kp'] = pd_params[0]
        self.PD_params[self.joint_names.index('joint3')]['kd'] = pd_params[1]

    def _apply_effort(self):

        for joint_name in self.joint_names:
            # Index of the joint in the current positions and target positions lists
            i = self.joint_names.index(joint_name)
            error = self.target_joint_positions[i] - self.joint_current_positions[i]

            effort = self._PD_Controller(joint_name=joint_name, error=error, kp=self.PD_params[i]['kp'], kd=self.PD_params[i]['kd'], ki=self.PD_params[i]['ki'], dt=self.dt, disturbance=self.gravity_compensation if joint_name == 'joint3' else 0.0)
            
            # Publish effort command to the joint
            self._publish_cmd_force(joint_name, effort)

    def _apply_velocity(self):
        for joint_name in self.joint_names:
            # Index of the joint in the current positions and target positions lists
            i = self.joint_names.index(joint_name)
            target = self.target_joint_positions[i]
            current = self.joint_current_positions[i]
            if target == 0.0:
                velocity = 0.0
            else:
                error = target - current
                velocity = self._PD_Controller(joint_name=joint_name, error=error, kp=self.PD_params[i]['kp'], kd=self.PD_params[i]['kd'], ki=self.PD_params[i]['ki'], dt=self.dt, disturbance=self.PD_params[i]['d'])
                
                if joint_name == self.tracked_joint_name:
                    self.get_logger().info(
                        f'Applying velocity control for joint "{joint_name}": '
                        f'current_velocity={self.joint_current_positions[i]:.4f}, '
                        f'target_velocity={self.target_joint_positions[i]:.4f}, '
                        f'error={error:.4f}, '
                        f'velocity_command={velocity:.4f}'
                )
            # Publish velocity command to the joint
            self._publish_cmd_velocity(joint_name, velocity)

    def _publish_cmd_velocity(self, joint_name: str, velocity: float):
        msg = Float64()
        msg.data = velocity
        self._velocity_pub[joint_name].publish(msg)

    # Publish effort command to the joint
    def _publish_cmd_force(self, joint_name: str, effort: float):
        msg = Float64()
        msg.data = effort
        self._effort_pub[joint_name].publish(msg)

    # PD controller
    def _PD_Controller(self, joint_name: str, error: float, kp: float, kd: float, ki: float, dt: float, disturbance: float = 0.0) -> float:
       
        if dt <= 0.0:
            return 0.0
        i = self.joint_names.index(joint_name)

        derivative = (error - self._pre_errors[i]) / dt
        integral = self._integrals[i] + error * dt
        self._integrals[i] = integral
        
        # effort = self.kp * error + self.kd * derivative
        effort = kp * error + kd * derivative + ki * integral
        self._pre_errors[i] = error

        return effort - disturbance


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
