import numpy as np

from .scara_kinematic_model import ScaraKinematicModel
import sympy as sp


class ScaraJacobianModel():

    @staticmethod
    def get_jacobian_matrices(positions = [0.0, 0.0, 0.0]):

        trans = ScaraKinematicModel.get_Trans_matrices_scara_robot(positions)
        ee = trans[-1]  # Transformation matrix of the end-effector
        Pe = ee[0:3, 3]  # Position of the end-effector
        joint_types = ScaraKinematicModel.get_joint_types()
        res = np.array([]).reshape(6, 0)  # Initialize an empty matrix for the Jacobian

        for i, T in enumerate(trans[0:-1]):  # Iterate through the transformation matrices of each joint (excluding the end-effector)
            # print(f"Transformation matrix for joint {i+1}:\n{T}\n")
            if joint_types[i] == 'revolute':
                Zi = T[0:3, 2]  # Extract the z-axis from the transformation matrix
                Pi = T[0:3, 3]  # Extract the position from the transformation matrix
                Jv = np.cross(Zi, Pe - Pi)  # Linear velocity part of the Jacobian
                # print(f"Jv for joint {i+1}:\n{Jv}\n")
                Jw = Zi  # Angular velocity part of the Jacobian
            elif joint_types[i] == 'prismatic':
                Pz = T[0:3, 2]  # Extract the z-axis from the transformation matrix
                Jv = Pz  # Linear velocity part of the Jacobian
                Jw = np.array([0.0, 0.0, 0.0])  # Angular velocity part of the Jacobian is zero for prismatic joints

            # Combine Jv and Jw into the full Jacobian for this joint
            
            Jp = np.concatenate((Jv, Jw)).reshape(-1, 1)  # Combine Jv and Jw into a single array
            res = np.hstack((res, Jp))  # Append the new column to the matrix res
        return res.astype(np.float64)

    @staticmethod
    def get_jacobian_linear_velocity(positions = [0.0, 0.0, 0.0]):
        J = ScaraJacobianModel.get_jacobian_matrices(positions)
        return J[0:3, :]

    @staticmethod
    def get_jacobian_angular_velocity(positions = [0.0, 0.0, 0.0]):
        J = ScaraJacobianModel.get_jacobian_matrices(positions)
        return J[3:6, :]

    @staticmethod
    def ee_to_joints_velocities(ee_velocities, current_joint_positions):
        # Placeholder for the actual kinematic calculations
        # In a real implementation, this would involve using the robot's kinematic model
        # to compute the joint velocities based on the end-effector velocities.
        joint_velocities = [1.0, 2.0, 1.0]  # Replace with actual calculations

        Jv = ScaraJacobianModel.get_jacobian_linear_velocity(current_joint_positions)
        # joint_velocities = inv(Jv)*x
        joint_velocities = np.linalg.pinv(Jv).dot(ee_velocities)  # Use the pseudo-inverse of Jv to compute joint velocities from end-effector velocities   

        return joint_velocities

    @staticmethod
    def joints_to_ee_velocities(joint_velocities, current_joint_positions):

        Jv = ScaraJacobianModel.get_jacobian_linear_velocity(current_joint_positions)
        ee_velocities = Jv.dot(joint_velocities)  # Compute end-effector velocities from joint velocities using the Jacobian
        return ee_velocities