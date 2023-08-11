import numpy as np
import pandas as pd

# Initial state: position (x, y, z) and velocity (vx, vy, vz)
state = np.zeros(6)

# Time step
dt = 0.01

# Covariance matrices for process and measurement noise
Q = np.diag([0.01, 0.01, 0.01,    # Process noise covariance for position
             0.001, 0.001, 0.001])  # Process noise covariance for velocity

R = np.diag([0.1, 0.1, 0.1])  # Measurement noise covariance for angular velocity

# Initial state covariance
P = np.diag([1.0, 1.0, 1.0,    # Position covariance
             1.0, 1.0, 1.0])  # Velocity covariance

def predict_state(A, x, P, Q):
    x = A @ x
    P = A @ P @ A.T + Q
    return x, P

def update_state(H, x, P, z, R):
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    P = (np.eye(len(x)) - K @ H) @ P
    return x, P

def update_position(acc, gyro):
    global state, P

    # Predict step: Using constant velocity motion model to predict the state
    A = np.eye(6)  # State transition matrix (constant velocity model)
    A[0:3, 3:6] = dt * np.eye(3)
    state, P = predict_state(A, state, P, Q)

    # Update step: Correct the state using angular velocity as a measurement
    z = gyro
    H = np.zeros((3, 6))
    H[:, 3:6] = dt * np.eye(3)
    state, P = update_state(H, state, P, z, R)

    # Update step: Correct the state using linear acceleration as a measurement
    z = acc
    H = np.zeros((3, 6))
    H[:, 3:6] = dt * np.eye(3)
    state, P = update_state(H, state, P, z, R)

    return state[0:3]  # Extract position (x, y, z) from the state vector

# Example usage with mock data
if __name__ == "__main__":
    # Mock data for linear acceleration and angular velocity (shape: (20, 3))
    # acc_data = np.zeros((20, 3))  # m/s^2
    # gyro_data = np.zeros((20, 3))  # rad/s

    # acc_data[:, 0] = 1

    data_file_path = 'data/gyro_calibration.csv'
    data = pd.read_csv(data_file_path)

    acc_data = data.filter(regex='acc').values
    gyro_data = data.filter(regex='gyr').values
    mag_data = data.filter(regex='mag').values

    for i in range(len(acc_data)):
        position = update_position(acc_data[i], gyro_data[i])
        print(f"Step {i+1}: Position = {position}")
