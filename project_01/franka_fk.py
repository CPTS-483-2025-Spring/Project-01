import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

def rotation_matrix_x(theta):
    """
    Compute the 3D rotation matrix around the x-axis.
    
    Parameters:
    theta (float): Rotation angle in radians.
    
    Returns:
    np.array: 3x3 rotation matrix.
    """
    return np.eye(3) # TODO

def rotation_matrix_y(theta):
    """
    Compute the 3D rotation matrix around the y-axis.
    
    Parameters:
    theta (float): Rotation angle in radians.
    
    Returns:
    np.array: 3x3 rotation matrix.
    """
    return np.eye(3) # TODO

def rotation_matrix_z(theta):
    """
    Compute the 3D rotation matrix around the z-axis.
    
    Parameters:
    theta (float): Rotation angle in radians.
    
    Returns:
    np.array: 3x3 rotation matrix.
    """
    return [np.array([
            [math.cos(theta), -math.sin(theta), 0],
            [math.sin(theta), math.cos(theta), 0],
            [0, 0, 1]
        ])]

def transformation_matrix(rotation_matrix, translation_vector):
    """
    Create a 4x4 homogeneous transformation matrix from a 3x3 rotation matrix and a 3x1 translation vector.
    
    Parameters:
    rotation_matrix (np.array): 3x3 rotation matrix.
    translation_vector (np.array): 3x1 translation vector.
    
    Returns:
    np.array: 4x4 homogeneous transformation matrix.
    """
    # Ensure the inputs are numpy arrays
    rotation_matrix = np.array(rotation_matrix)
    translation_vector = np.array(translation_vector)
    
    # TODO: Create the transformation matrix
    transformation_matrix = np.eye(4)  # Start with a 4x4 identity matrix
    
    return transformation_matrix

def forward_kinematics_franka(joint_angles):
    """
    Compute the forward kinematics for franka arm (panda_ee frame) using transformation matrices.
    
    Parameters:
    joint_angles (list): List of joint angles in radians [theta1, ..., theta7].
    
    Returns:
    np.array: transformation matrix of the end-effector.
    """
    
    # Rotation matrices for each joint
    theta1, theta2, theta3, theta4, theta5, theta6, theta7 = joint_angles
    R01 = rotation_matrix_z(theta1)  # First joint rotates around z-axis
    # TODO: complete all rotation matrices
    
    print("R34:\n", R34)
    print("R78:\n", R78)
    
    # Translation vectors for each link
    p01 = np.array([0, 0, 0.333])  # First link translates along z-axis
    # TODO: complete all translation vectors

    # TODO: complete all transformation matrix
    T01 = transformation_matrix(R01, p01)

    print("T01:\n", T01)
    print("T12:\n", T12)
    print("T23:\n", T23)
    print("T34:\n", T34)
    print("T45:\n", T45)
    print("T56:\n", T56)
    print("T67:\n", T67)
    print("T78:\n", T78)
    
    # TODO: Compute the end-effector position
    ee_T = np.eye(4)
    
    return ee_T


class FKNode(Node):

    def __init__(self):
        super().__init__('fk_calculation_node')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',  # Replace with your topic name
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # get joint angles
        joint_names = msg.name
        print(joint_names)
        joint_names_in_order = ["panda_joint1",
                                "panda_joint2",
                                "panda_joint3",
                                "panda_joint4",
                                "panda_joint5",
                                "panda_joint6",
                                "panda_joint7"]
        indices = [joint_names.index(name) for name in joint_names_in_order]
        joint_angles = [msg.position[i] for i in indices]
        ee_T = forward_kinematics_franka(joint_angles)  # compute FK
        ee_round = np.round(ee_T, 3)
        self.get_logger().info(f'Joint Angles 1-7 : {joint_angles}')
        self.get_logger().info(f'EE Transformation Matrix:: {ee_round}')


def main(args=None):
    rclpy.init(args=args)
    calculation_node = FKNode()
    rclpy.spin(calculation_node)
    calculation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # ee = forward_kinematics_franka([0, 0, 0, -1.571, 0, 0, 0])
    # print("Transformation Matrix:\n", np.round(ee, 3))
    main()