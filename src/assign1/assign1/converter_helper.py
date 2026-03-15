import math

from geometry_msgs.msg import Transform

class ConverterHelper:
	@staticmethod
	def normalize(qx, qy, qz, qw):
		norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
		if norm == 0.0:
			return 0.0, 0.0, 0.0, 1.0
		return qx / norm, qy / norm, qz / norm, qw / norm

	@staticmethod
	def quat_to_rotation_array(qx, qy, qz, qw):
		"""
            Convert a quaternion to a 3x3 rotation matrix.
            This function takes quaternion components and converts them into a rotation matrix
            representation. The quaternion is first normalized to ensure valid rotation representation,
            then the rotation matrix elements are computed using the standard quaternion-to-rotation
            matrix conversion formulas.
            Args:
                qx (float): The x component of the quaternion.
                qy (float): The y component of the quaternion.
                qz (float): The z component of the quaternion.
                qw (float): The w (scalar) component of the quaternion.
            Returns:
                list[list[float]]: A 3x3 rotation matrix represented as a list of lists,
                                where each inner list represents a row of the matrix.
                                The matrix can be used to rotate 3D vectors.
            Note:
                The input quaternion is normalized before conversion to ensure it represents
                a valid rotation.
            Example:
			>>> qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0  # Identity quaternion
			>>> rotation_matrix = quat_to_rotation_array(qx, qy, qz, qw)
			>>> # Returns identity rotation matrix: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
		"""

		qx, qy, qz, qw = ConverterHelper.normalize(qx, qy, qz, qw)

		r11 = 1.0 - 2.0 * (qy * qy + qz * qz)
		r12 = 2.0 * (qx * qy - qz * qw)
		r13 = 2.0 * (qx * qz + qy * qw)

		r21 = 2.0 * (qx * qy + qz * qw)
		r22 = 1.0 - 2.0 * (qx * qx + qz * qz)
		r23 = 2.0 * (qy * qz - qx * qw)

		r31 = 2.0 * (qx * qz - qy * qw)
		r32 = 2.0 * (qy * qz + qx * qw)
		r33 = 1.0 - 2.0 * (qx * qx + qy * qy)

		return [
			[r11, r12, r13],
			[r21, r22, r23],
			[r31, r32, r33],
	]

	@staticmethod
	def rot_to_quat(rotation):
		"""
		    Convert a 3x3 rotation matrix to a quaternion representation.
            This function implements the standard algorithm for converting a rotation matrix
            to a quaternion (qx, qy, qz, qw), selecting the most numerically stable formula
            based on the trace and diagonal elements of the rotation matrix.
            Args:
                rotation (list or array-like): A 3x3 rotation matrix where:
                    - rotation[i][j] represents the element at row i, column j
            Returns:
                tuple: A quaternion in the form (qx, qy, qz, qw) where:
                    - qx, qy, qz (float): The vector components of the quaternion
                    - qw (float): The scalar component of the quaternion
            Notes:
                - The function uses different calculation paths depending on the trace
                and diagonal values to ensure numerical stability
                - The input rotation matrix should be a valid orthogonal rotation matrix
		"""
		
		r11 = rotation[0][0]
		r12 = rotation[0][1]
		r13 = rotation[0][2]
		r21 = rotation[1][0]
		r22 = rotation[1][1]
		r23 = rotation[1][2]
		r31 = rotation[2][0]
		r32 = rotation[2][1] 
		r33 = rotation[2][2]

        
		tr = r11 + r22 + r33
		if tr > 0:
			S = math.sqrt(tr + 1.0) * 2
			qw = 0.25 * S
			qx = (r32 - r23) / S
			qy = (r13 - r31) / S
			qz = (r21 - r12) / S
		elif (r11 > r22) and (r11 > r33):
			S = math.sqrt(1.0 + r11 - r22 - r33) * 2
			qw = (r32 - r23) / S
			qx = 0.25 * S
			qy = (r12 + r21) / S
			qz = (r13 + r31) / S
		elif r22 > r33:
			S = math.sqrt(1.0 + r22 - r11 - r33) * 2
			qw = (r13 - r31) / S
			qx = (r12 + r21) / S
			qy = 0.25 * S
			qz = (r23 + r32) / S
		else:
			S = math.sqrt(1.0 + r33 - r11 - r22) * 2
			qw = (r21 - r12) / S
			qx = (r13 + r31) / S
			qy = (r23 + r32) / S
			qz = 0.25 * S

		return qx, qy, qz, qw

	@staticmethod
	def quat_to_euler(qx, qy, qz, qw, degrees=False):
		qx, qy, qz, qw = ConverterHelper.normalize(qx, qy, qz, qw)

		sinr_cosp = 2.0 * (qw * qx + qy * qz)
		cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
		roll = math.atan2(sinr_cosp, cosr_cosp)

		sinp = 2.0 * (qw * qy - qz * qx)
		if abs(sinp) >= 1.0:
			pitch = math.copysign(math.pi / 2.0, sinp)
		else:
			pitch = math.asin(sinp)

		siny_cosp = 2.0 * (qw * qz + qx * qy)
		cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
		yaw = math.atan2(siny_cosp, cosy_cosp)

		if degrees:
			return {
				"roll": math.degrees(roll),
				"pitch": math.degrees(pitch),
				"yaw": math.degrees(yaw),
			}

		return {
			"roll": roll,
			"pitch": pitch,
			"yaw": yaw,
		}
	
	@staticmethod
	def transform_matrix_to_transform_msg(transform_matrix):       
		"""
		Convert a 4x4 transformation matrix to a geometry_msgs/Transform message.
		This function takes a 4x4 homogeneous transformation matrix, extracts the translation
		Arguments:
		- transform_matrix (list or array-like): A 4x4 transformation matrix where
			transform_matrix[i][j] represents the element at row i, column j.
		Returns:
		- geometry_msgs.msg.Transform: A ROS Transform message containing the translation and rotation
			extracted from the input transformation matrix. The translation is represented as a Vector3,
			and the rotation is represented as a quaternion (x, y, z, w).
		"""

		translation = transform_matrix[0:3, 3]
		rotation = ConverterHelper.rot_to_quat(transform_matrix[0:3, 0:3])

		res = Transform()
		res.translation.x = translation[0]
		res.translation.y = translation[1]
		res.translation.z = translation[2]
		res.rotation.x = rotation[0]
		res.rotation.y = rotation[1]
		res.rotation.z = rotation[2]
		res.rotation.w = rotation[3]    
		return res

