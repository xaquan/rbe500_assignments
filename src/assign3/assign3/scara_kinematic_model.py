
import numpy as np
from .converter_helper import ConverterHelper
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
import sympy as sp

class ScaraKinematicModel():
    @staticmethod
    def get_robot_configuration():

        # Robot dimensions
        d1 = 0.5
        a1 = 0.45
        a2 = 0.35
        d3 = 0.3
        # return as object for better readability
        return {
            'd1': d1,
            'a1': a1,
            'a2': a2,
            'd3': d3,
        }

    @staticmethod
    def get_joint_types():
        res = ["revolute", "revolute", "prismatic"]
        return res

    @staticmethod
    def wrap_to_pi(a):
        """Wrap angle(s) to [-pi, pi)."""
        a = np.asarray(a, dtype=float)
        return (a + np.pi) % (2*np.pi) - np.pi
    
    # Calculate the homogeneous transformation matrix from DH parameters
    @staticmethod
    def dh_standard(theta, d, a, alpha):
        """
        Computes the Denavit-Hartenberg (DH) standard transformation matrix.
        This method calculates the 4x4 homogeneous transformation matrix based on 
        the standard DH convention parameters. The transformation represents the 
        position and orientation of a joint frame relative to the previous frame.
        Args:
            theta (float): Joint angle (rotation about Z-axis) in radians.
            d (float): Link offset (translation along Z-axis).
            a (float): Link length (translation along X-axis).
            alpha (float): Link twist (rotation about X-axis) in radians.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix of the form:
                [[cos(θ), -sin(θ)cos(α), sin(θ)sin(α), a*cos(θ)],
                 [sin(θ), cos(θ)cos(α), -cos(θ)sin(α), a*sin(θ)],
                 [0, sin(α), cos(α), d],
                 [0, 0, 0, 1]]
        Note:
            Trigonometric values are rounded to 10 decimal places to avoid 
            floating-point precision errors.
        """

        ct, st = sp.cos(theta), sp.sin(theta)
        ca, sa = sp.cos(alpha), sp.sin(alpha)
        return sp.Matrix([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d],
            [0,  0,    0,  1]
        ])

    @staticmethod
    def get_Trans_matrices_scara_robot_symbolic():
        """
        Computes the symbolic transformation matrices for each joint of a SCARA robot.
        This method calculates the 4x4 homogeneous transformation matrices representing the pose of each joint frame
        relative to the base frame of a SCARA robot. The transformations are derived from the Denavit-Hartenberg (DH) parameters, which are defined symbolically for the robot's joints and links.
        Returns:
            list of sp.Matrix: A list of 4x4 symbolic homogeneous transformation matrices for each joint of the SCARA robot. Each matrix is of the form:
                [[T11, T12, T13, Px],
                 [T21, T22, T23, Py],
                 [T31, T32, T33, Pz],
                 [0,   0,   0,   1]]
            where Tij represents the rotation components and (Px, Py, Pz) represents the position of the joint frame.
        """

        # Cast thetas value to variable names for better readability
        q1, q2, q3 = sp.symbols('q1 q2 q3')

        #Physical mesurements of the robot
        d1, d3 = sp.symbols('d1 d3')
        a1, a2 = sp.symbols('a1 a2')

        DH = [
            (q1, d1, a1, 0),
            (q2, 0, a2, 0),
            (0, -(d3 + q3), 0, sp.pi),
        ]

        # Calculate the forward kinematics using the DH parameters
        T = sp.eye(4)
        res = [T]
        for theta, d, a, alpha in DH:
            # Calculate the homogeneous transformation matrix for each joint and multiply them together
            dh_std = ScaraKinematicModel.dh_standard(theta, d, a, alpha)
            T = T * dh_std
            res.append(T)
        return res

    @staticmethod
    def get_Trans_matrices_scara_robot(positions = [0.0, 0.0, 0.0]):
        """
        Computes the numerical transformation matrices for each joint of a SCARA robot based on given joint positions.
        This method calculates the 4x4 homogeneous transformation matrices representing the pose of each joint frame
        relative to the base frame of a SCARA robot. The transformations are derived from the Denavit-Hartenberg (DH) parameters, which are defined based on the robot's joint angles and link dimensions.
        Args:
            positions (ndarray): A 1D array of 3 float values representing the joint angles [q1, q2, q3] in radians. 
            Each angle corresponds to a joint of the SCARA robot manipulator.
        Returns:   
            list of np.ndarray: A list of 4x4 homogeneous transformation matrices for each joint of the SCARA robot. Each matrix is of the form:
                [[T11, T12, T13, Px],
                 [T21, T22, T23, Py],
                 [T31, T32, T33, Pz],
                 [0,   0,   0,   1]]
            where Tij represents the rotation components and (Px, Py, Pz) represents the position of the joint frame. 
            The matrices are returned as NumPy arrays of type float64 for numerical computations.
        """
        if len(positions) != 3:
            raise ValueError("Expected 3 joint angles, got {}".format(len(positions)))

        T_final_symbolic = ScaraKinematicModel.get_Trans_matrices_scara_robot_symbolic()
        res = []
        for T in T_final_symbolic:
            T_num = T.subs({
                'q1': positions[0],
                'q2': positions[1],
                'q3': positions[2],
                'd1': ScaraKinematicModel.get_robot_configuration()['d1'],
                'a1': ScaraKinematicModel.get_robot_configuration()['a1'],
                'a2': ScaraKinematicModel.get_robot_configuration()['a2'],
                'd3': ScaraKinematicModel.get_robot_configuration()['d3'],
            })
            res.append(np.array(T_num).astype(np.float64))
        return res


    @staticmethod
    def get_Trans_end_effector_scara_robot_symbolic():

        """
            Computes the forward kinematics transformation matrix for a SCARA robot using symbolic variables.
            This method calculates the 4x4 homogeneous transformation matrix representing the end-effector pose
            relative to the base frame of a SCARA robot. The transformation is derived from the Denavit-Hartenberg (DH) parameters, which are defined symbolically for the robot's joints and links.
            Returns:
                sp.Matrix: A 4x4 symbolic homogeneous transformation matrix of the form:
                    [[T11, T12, T13, Px],
                     [T21, T22, T23, Py],
                     [T31, T32, T33, Pz],
                     [0,   0,   0,   1]]
                where Tij represents the rotation components and (Px, Py, Pz) represents the position of the end-effector."""
                
        # # Cast thetas value to variable names for better readability
        # q1, q2, q3 = sp.symbols('q1 q2 q3')

        # #Physical mesurements of the robot
        # d1, d3 = sp.symbols('d1 d3')
        # a1, a2 = sp.symbols('a1 a2')

        # DH = [
        #     (q1, d1, a1, 0),
        #     (q2, 0, a2, 0),
        #     (0, -(d3 + q3), 0, sp.pi),
        # ]

        # # Calculate the forward kinematics using the DH parameters
        # T = sp.eye(4)
        
        # for theta, d, a, alpha in DH:
        #     # Calculate the homogeneous transformation matrix for each joint and multiply them together
        #     dh_std = ScaraKinematicModel.dh_standard(theta, d, a, alpha)
        #     T = T @ dh_std

        trans = ScaraKinematicModel.get_Trans_matrices_scara_robot_symbolic()
        return trans[-1]
    
    @staticmethod
    def print_symbolic_fk():
        """
            Prints the symbolic forward kinematics transformation matrix for the SCARA robot.
            This method computes the symbolic forward kinematics transformation matrix for a 
            SCARA robot and prints it in a readable format. The transformation matrix is derived 
            from the Denavit-Hartenberg (DH)
        """
        T_symbolic = ScaraKinematicModel.get_Trans_end_effector_scara_robot_symbolic()
        sp.pprint(T_symbolic)

    
    @staticmethod
    def forward_kinematics_scara_robot(positions):
        """
            Calculate the forward kinematics transformation matrix for a SCARA robot manipulator.
            This function computes the homogeneous transformation matrix representing the end-effector pose
            relative to the base frame of a SCARA robot. The transformation is derived from the 
            Denavit-Hartenberg (DH) parameters, which are defined based on the robot's joint angles and link dimensions.
            Args:
                positions (ndarray): A 1D array of 3 float values representing the joint angles [q1, q2, q3] in radians. 
                Each angle corresponds to a joint of the SCARA robot manipulator.
            Returns:
                ndarray: A 4x4 homogeneous transformation matrix of the form:
                    [[T11, T12, T13, Px],
                     [T21, T22, T23, Py],
                     [T31, T32, T33, Pz],
                     [0,   0,   0,   1]]
                where Tij represents the rotation components and (Px, Py, Pz) represents the position of the end-effector. 
                The matrix is returned as a NumPy array of type float64 for
        """
        if len(positions) != 3:
            raise ValueError("Expected 3 joint angles, got {}".format(len(positions)))

        T_final_symbolic = ScaraKinematicModel.get_Trans_end_effector_scara_robot_symbolic()

        T_final = T_final_symbolic.subs({
            'q1': positions[0],
            'q2': positions[1],
            'q3': positions[2],
            'd1': ScaraKinematicModel.get_robot_configuration()['d1'],
            'a1': ScaraKinematicModel.get_robot_configuration()['a1'],
            'a2': ScaraKinematicModel.get_robot_configuration()['a2'],
            'd3': ScaraKinematicModel.get_robot_configuration()['d3'],
        })

        return np.array(T_final).astype(np.float64)

    @staticmethod
    def inverse_kinematics_scara_robot(pose):
        """
            Calculate the inverse kinematics solution for a SCARA robot manipulator.
            This function computes the joint angles (q1, q2, q3) required to achieve
            a desired end-effector position and orientation. It uses geometric approach to solve
            the inverse kinematics problem for a SCARA robot with the specified robot parameters.
            Args:
                pose (geometry_msgs.msg.Pose): The desired end-effector pose as a 3D position and orientation.
            Returns:
                ndarray: A 1D array of 3 float values representing the joint angles [q1, q2, q3] in radians.
                        Each angle corresponds to a joint of the SCARA robot manipulator.
        """

        # Robot dimensions
        d1 = ScaraKinematicModel.get_robot_configuration()['d1']
        a1 = ScaraKinematicModel.get_robot_configuration()['a1']
        a2 = ScaraKinematicModel.get_robot_configuration()['a2']
        d3 = ScaraKinematicModel.get_robot_configuration()['d3']

        # Convert quaternion to rotation matrix
        rotation = Rotation.from_quat([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]).as_matrix()

        r11 = rotation[0][0]
        r12 = rotation[0][1]
        r13 = rotation[0][2]
        r21 = rotation[1][0]
        r22 = rotation[1][1]
        r23 = rotation[1][2]
        r31 = rotation[2][0]
        r32 = rotation[2][1] 
        r33 = rotation[2][2]


        # Calculate wrist center position        
        px = pose.position.x
        py = pose.position.y
        pz = d1  # Adjust for the end-effector orientation and d3 offset

        # print(f"Joint 3 position: ({px}, {py}, {pz})")

        r1 = np.sqrt(px**2 + py**2)

        # print(f"r1: {r1}")

        # Calculate q1, q2, q3 using geometric approach

        """ Solvig for q1
            q1 = alpha - beta        
            alpha = atan2(cos(alpha), sin(alpha))
            beta = atan2(cos(beta), sin(beta))
            
            cos(alpha) = atan2(py, px)
            sin(alpha) = sqrt(1 - cos(alpha)^2)
            cos(beta) = (a1^2 + r1^2 - a2^2) / (2*a1*r1)
            sin(beta) = sqrt(1 - cos(beta)^2)
        """

        cos_alpha = px / r1
        cos_alpha = np.clip(cos_alpha, -1, 1)  # Ensure the value is within the valid range for arccos
        sin_alpha = np.sqrt(1 - cos_alpha**2)

        # print(f"cos_alpha: {cos_alpha}, sin_alpha: {sin_alpha}")

        cos_beta = (a1**2 + r1**2 - a2**2) / (2*a1*r1)
        cos_beta = np.clip(cos_beta, -1, 1)  # Ensure the value is within the valid range for arccos
        sin_beta = np.sqrt(1 - cos_beta**2)

        # print(f"cos_beta: {cos_beta}, sin_beta: {sin_beta}")

        alpha_1 = np.arctan2(sin_alpha, cos_alpha)
        alpha_2 = np.arctan2(-sin_alpha, cos_alpha)
        beta_1 = np.arctan2(sin_beta, cos_beta)
        beta_2 = np.arctan2(-sin_beta, cos_beta)

        # print(f"alpha_1: {alpha_1}, alpha_2: {alpha_2}")
        # print(f"beta_1: {beta_1}, beta_2: {beta_2}")

        q1_candidates = np.array([alpha_1 - beta_1, alpha_1 - beta_2, alpha_2 - beta_1, alpha_2 - beta_2])
        k = np.argmin(np.abs(ScaraKinematicModel.wrap_to_pi(q1_candidates)))
        # q1 = q1_candidates[k]

        # Solving for q2
        # cos(q2) = -(a1^2 + a2^2 - r1^2) / (2*a1*a2)
        # sin(q2) = sqrt(1 - cos(q2)^2)
        # q2 = atan2(+-sin(q2), cos(q2))

        cos_q2 = -(a1**2 + a2**2 - r1**2) / (2*a1*a2)
        cos_q2 = np.clip(cos_q2, -1, 1)  # Ensure the value is within the valid range for arccos

        sin_q2 = np.sqrt(1 - cos_q2**2)

        q2_1 = np.arctan2(sin_q2, cos_q2)
        q2_2 = np.arctan2(-sin_q2, cos_q2)

        q2_candidates = np.array([q2_1, q2_2])
        
        k = np.argmin(np.abs(ScaraKinematicModel.wrap_to_pi(q2_candidates)))
        # q2 = q2_candidates[k]

        # print(f"Found angles: {aangles}")

        # Solving for q3
        # q3 = d1 - pose.position.z - d3        
        q3 = d1 - pose.position.z - d3

        
        aangles = ScaraKinematicModel.find_combinations(pose, q1_candidates, q2_candidates, q3)
        # return array float64 for better compatibility with ROS messages
        # return np.array([q1, q2, q3]).astype(np.float64)
        return aangles.astype(np.float64)
   

    @staticmethod
    def find_combinations(pose, q1_candidates, q2_candidates, q3):
        """
            Verifies the correctness of the inverse kinematics solution by comparing the forward kinematics result with the desired pose.
            This function computes the forward kinematics using the provided joint angles and compares the resulting end-effector pose with the original target pose. It checks if the position and orientation of the computed pose are within a specified tolerance of the target pose, indicating whether the inverse kinematics solution is valid.
            Args:
                pose (geometry_msgs.msg.Pose): The original target end-effector pose that was used to compute the inverse kinematics solution.
                joint_angles (ndarray): A 1D array of joint angles [q1, q2, q3] that were computed as the inverse kinematics solution.
        """
        # find combination closest to pose

        for q1 in q1_candidates:
            for q2 in q2_candidates:
                joint_angles = np.array([q1, q2, q3])
                pose_computed = ScaraKinematicModel.forward_kinematics_scara_robot(joint_angles)
                position_error = np.linalg.norm(pose_computed[:3, 3] - np.array([pose.position.x, pose.position.y, pose.position.z]))
                orientation_error = np.linalg.norm(pose_computed[:3, :3] - ConverterHelper.quat_to_rotation_array(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
                if position_error < 1e-3 and orientation_error < 1e-3:
                    return joint_angles
                
        
