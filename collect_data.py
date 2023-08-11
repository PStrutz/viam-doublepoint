from skinematics.imus import IMU_Base
from touch_sdk import Watch
from touch_sdk.watch import SensorFrame
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime
import quaternion

class MyWatch(Watch):
    def __init__(self, name_filter=None):
        super().__init__()
        self.custom_data = 'data/data_imu_wit.txt'
        self.calibration_index = 0
        self.reference_orientation = np.ndarray((200, 4))
        self.rotation_matrix = np.ndarray((3, 3))

    def collect_calibration_data(self, sensors):
        self.reference_orientation[self.calibration_index] = sensors.orientation
        self.calibration_index += 1
        if self.calibration_index == 200:
            self.calibrate_imu()
    
    def calibrate_imu(self):
        # Define the initial orientation quaternion 
        initial_quaternion = np.mean(self.reference_orientation, axis=0)
        initial_quaternion = np.roll(initial_quaternion, 1, axis=0)
        print(initial_quaternion)
        initial_quaternion = quaternion.from_float_array(initial_quaternion)
        # Define the desired gravity vector in the sensor's coordinate frame
        desired_gravity = np.array([0, 0, -9.8])  # [accel_x, accel_y, accel_z] in m/s^2

        # Rotate the desired gravity vector to the global coordinate frame using the initial quaternion
        rotated_gravity = (initial_quaternion * quaternion.quaternion(0, *desired_gravity) * initial_quaternion.inverse()).vec

        # Calculate the rotation matrix that aligns the rotated gravity vector with the Z-axis
        self.rotation_matrix = np.array([
            rotated_gravity / np.linalg.norm(rotated_gravity),  # Z-axis
            np.array([0, 0, 1]),  # Y-axis
            np.cross(rotated_gravity, [0, 0, 1]) / np.linalg.norm(np.cross(rotated_gravity, [0, 0, 1]))  # X-axis
        ])

        print(self.rotation_matrix)






    def on_tap(self):
        self.trigger_haptics(1.0, 20)
        
        
    def on_sensors(self, sensors):
        if self.calibration_index < 200:
            print("Calibrating... Please stay still.")
            self.collect_calibration_data(sensors)
        else:
            if sensors.magnetic_field:
                mag = sensors.magnetic_field
            else:
                mag = [None, None, None]

            # Correct the accelerometer measurements using the rotation matrix
            acc = np.dot(sensors.acceleration, self.rotation_matrix.T)
            
            data = {
                "acc_x": acc[0],
                "acc_y": acc[1],
                "acc_z": acc[2],
                "gyr_x": sensors.angular_velocity[0],
                "gyr_y": sensors.angular_velocity[1],
                "gyr_z": sensors.angular_velocity[2],
                "mag_x": mag[0],
                "mag_y": mag[1],
                "mag_z": mag[2]
            }
            file_path = 'data/gyro_calibration.csv'
            # create headers the first time data is read in
            file_is_empty = os.stat(file_path).st_size == 0
            df = pd.DataFrame(data, index=[0])
            #print(datetime.now())
            df.to_csv(file_path, mode='a', header=file_is_empty)
            print(data['acc_x'], data['acc_y'], data['acc_z'])


watch = MyWatch()
watch.start()
print('done')









