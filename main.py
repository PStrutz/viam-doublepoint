from imu import * 
import numpy as np
from numpy.linalg import inv, norm

import data_receiver
from mathlib import *
from plotlib import *

from skinematics.imus import IMU_Base
from touch_sdk import Watch
from touch_sdk.watch import SensorFrame
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime


class MyWatch(Watch):
    def __init__(self, name_filter=None):
        super().__init__()
        self.custom_data = 'data/gyro_calibration.csv'


    def on_tap(self):
        self.trigger_haptics(1.0, 20)
        self.get_position()
        
        
    def on_sensors(self, sensors):
        if sensors.magnetic_field:
            mag = sensors.magnetic_field
        else:
            mag = [None, None, None]
        data = {
            "acc_x": sensors.acceleration[0] - 0.2,
            "acc_y": sensors.acceleration[1] + 0.34,
            "acc_z": sensors.acceleration[2] - 9.7,
            "gyr_x": sensors.angular_velocity[0],
            "gyr_y": sensors.angular_velocity[1],
            "gyr_z": sensors.angular_velocity[2],
            "mag_x": mag[0],
            "mag_y": mag[1],
            "mag_z": mag[2]
        }
        file_path = self.custom_data
        # create headers the first time data is read in
        file_is_empty = os.stat(file_path).st_size == 0
        df = pd.DataFrame(data, index=[0])
        #print(datetime.now())
        df.to_csv(file_path, mode='a', header=False)
        

        #print("wrote sensor data to file")

    def get_position(self):
        tracker = IMUTracker(sampling=100)
        data = receive_data(self.custom_data, 'file')    # toggle data source between 'tcp' and 'file' here

        print('initializing...')
        init_list = tracker.initialize(data[5:30])

        print('--------')
        print('processing...')
        
        # EKF step
        a_nav, orix, oriy, oriz = tracker.attitudeTrack(data[30:], init_list)

        # Acceleration correction step
        a_nav_filtered = tracker.removeAccErr(a_nav, filter=False)

        # ZUPT step
        v = tracker.zupt(a_nav_filtered, threshold=0.2)

        # Integration Step
        p = tracker.positionTrack(a_nav_filtered, v)
        plot3D([[p, 'position']])
    

watch = MyWatch()
watch.start()
print('done')





