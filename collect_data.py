from skinematics import quat
from skinematics.imus import IMU_Base
from touch_sdk import Watch
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

class WatchIMU(IMU_Base):
    """Concrete class based on abstract base class IMU_Base """    
    
    def get_data(self, in_file=None, in_data=None):
        
        try:
            # The sampling rate has to be provided externally
            rate = 100
            
            # Get the data, and label them
            data = pd.read_csv(in_file)
            print(data.head())

            #data.columns = ['acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z', 'mag_x', 'mag_y', 'mag_z']
            
        except FileNotFoundError:
            print('{0} does not exist!'.format(in_file))
            return -1
   
        # Extract the columns that you want, and pass them on
        in_data = {'rate':rate,
               'acc':   data.filter(regex='acc').values,
               'omega': data.filter(regex='gyr').values,
               'mag':   data.filter(regex='mag').values}
        self._set_data(in_data)

class MyWatch(Watch):
    def __init__(self, name_filter=None):
        super().__init__()
        self.file_path = 'data/watch_data.csv'
        self.calibration_index = 0
        self.calibrate_orientation = np.ndarray((200, 4))
        self.rotation_matrix = np.ndarray((3, 3))
        self.initial_orientation
        
        # create/empty file to save data to
        f = open(self.file_path, 'w')  # create new file or overwrite existing file
        f.close()

        
    
    def collect_calibration_data(self, sensors):
        self.calibrate_orientation[self.calibration_index] = sensors.orientation
        self.calibration_index += 1
        if self.calibration_index == 200:
            self.calibrate_imu()
    
    def calibrate_imu(self):
        # Define the initial orientation quaternion 
        orientation_quaternion = np.mean(self.calibrate_orientation, axis=0)
        print(np.roll(orientation_quaternion, shift=1))
        rotation_matrix = quat.convert(np.roll(orientation_quaternion, shift=1))
        self.rotation_matrix = rotation_matrix
        self.initial_orientation = orientation_quaternion


    def quaternion_to_rotation_matrix(self, quaternion):
        q1, q2, q3, q0 = quaternion
        rotation_matrix = np.array([
            [1 - 2*q2**2 - 2*q3**2, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],
            [2*q1*q2 + 2*q0*q3, 1 - 2*q1**2 - 2*q3**2, 2*q2*q3 - 2*q0*q1],
            [2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q0*q1, 1 - 2*q1**2 - 2*q2**2]
        ])
        return rotation_matrix

    def on_tap(self):
        self.trigger_haptics(1.0, 20)
        
    def on_sensors(self, sensors):
        if self.calibration_index < 200:
            print("Calibrating... Please stay still.")
            self.collect_calibration_data(sensors)
            initial_orientation = self.rotation_matrix
            initial_position = np.r_[0, 0, 0]
            
        else:
            if sensors.magnetic_field:
                mag = sensors.magnetic_field
            else:
                mag = [None, None, None]

            # Correct the accelerometer measurements using the rotation matrix
            acc = np.dot(self.rotation_matrix, sensors.acceleration)
            
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
            
            # create headers the first time data is read in
            file_is_empty = os.stat(self.file_path).st_size == 0
            df = pd.DataFrame(data, index=[0])
            df.to_csv(self.file_path, mode='a', header=file_is_empty)
            print(data['acc_x'], data['acc_y'], data['acc_z'])
            my_sensor = WatchIMU(in_file=self.file_path, R_init=initial_orientation, pos_init=initial_position)
            my_sensor.set_qtype("analytical")
            my_sensor.calc_position()

        
            
            


watch = MyWatch()
watch.start()
print('done')


