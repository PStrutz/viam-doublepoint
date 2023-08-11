import numpy as np


def quaternion_to_rotation_matrix(quaternion):
    q1, q2, q3, q0 = quaternion
    rotation_matrix = np.array([
        [1 - 2*q2**2 - 2*q3**2, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],
        [2*q1*q2 + 2*q0*q3, 1 - 2*q1**2 - 2*q3**2, 2*q2*q3 - 2*q0*q1],
        [2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q0*q1, 1 - 2*q1**2 - 2*q2**2]
    ])
    return rotation_matrix

def align_acceleration_with_gravity(orientation_quaternion, local_acceleration):
    rotation_matrix = quaternion_to_rotation_matrix(orientation_quaternion)
    
    # Transform acceleration to local frame
    local_acceleration_transformed = np.dot(np.linalg.inv(rotation_matrix), local_acceleration)
    
    # Adjust for gravity
    gravity_direction = np.array([0, 0, 1])  # Assuming gravity points along the positive z-axis
    gravity_magnitude = np.linalg.norm(local_acceleration_transformed)
    adjusted_local_acceleration = local_acceleration_transformed - gravity_magnitude * gravity_direction
    
    # Transform adjusted acceleration back to global frame
    global_acceleration = np.dot(rotation_matrix, adjusted_local_acceleration)
    
    return global_acceleration

# Given orientation quaternion
orientation_quaternion = np.array([-0.01129399985074997, -0.005264999810606241, 0.03361000120639801, 0.9993569850921631])  # Example quaternion (w, x, y, z)

# Given measured acceleration vector in local frame
local_acceleration = np.array([0.10055647045373917, -0.27054479718208313, 9.761159896850586])  # Example local acceleration (x, y, z)

rotation_matrix = quaternion_to_rotation_matrix(orientation_quaternion)
corrected_acceleration = np.dot(np.linalg.inv(rotation_matrix), local_acceleration)
print(corrected_acceleration)
global_acceleration = align_acceleration_with_gravity(orientation_quaternion, local_acceleration)
print("Global Acceleration:", global_acceleration)
