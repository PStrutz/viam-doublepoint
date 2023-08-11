import numpy as np
import pandas as pd
from ahrs.filters import Madgwick

# Initial position, velocity, and orientation
position = np.array([0.0, 0.0, 0.0])
velocity = np.array([0.0, 0.0, 0.0])
orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion (scalar first)

# Time step
dt = 1/100

def update_position(Q, acc, gyro, mag):
    global position, velocity, orientation

    # Madgwick filter to estimate orientation (quaternion)
    orientation = Madgwick(orientation).updateMARG(gyr=gyro, acc=acc, mag=mag, dt=dt)
    

    # Convert linear acceleration from local frame to global frame
    acc_global = Madgwick(orientation).transform_accel(acc)

    # Update velocity using linear acceleration
    velocity += acc_global * dt

    # Integrate velocity to get position
    position += velocity * dt

    return position

if __name__ == "__main__":
    num_samples = 20
    # Mock data for linear acceleration (in local frame), angular velocity (in rad/s), and magnetometer data (normalized)
    # acc_data = np.ndarray(shape=(num_samples,3), dtype=float, order='F') + 1
    # gyro_data = np.ndarray(shape=(num_samples,3), dtype=float, order='F') + 1 
    # mag_data = np.ndarray(shape=(num_samples,3), dtype=float, order='F') + 1

    data_file_path = 'data/watch_data_right.csv'
    data = pd.read_csv(data_file_path)

    acc_data = data.filter(regex='acc').values
    gyro_data = data.filter(regex='gyr').values
    mag_data = data.filter(regex='mag').values
    print(acc_data.shape)
    print(type(acc_data))
    print(acc_data)
    madgwick = Madgwick()
    Q = np.tile([1., 0., 0., 0.], (len(gyro_data), 1)) # Allocate for quaternions

    for t in range(1, len(acc_data)):
        Q[t] = madgwick.updateMARG(Q[t-1], gyr=gyro_data[t], acc=acc_data[t], mag=mag_data[t])

    
    print(Q.shape)
    print(Q)

    # not working atm
    # madgwick = Madgwick(gyr=gyro_data, acc=acc_data, mag=mag_data)
    # print(madgwick.Q.shape)
    # print(madgwick.Q)