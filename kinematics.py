from skinematics.imus import IMU_Base
from touch_sdk import Watch
from touch_sdk.watch import SensorFrame
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
    

if __name__ == '__main__':   
    
    initial_orientation = np.array([[-0.145386 , 0, 0], [0, -0.310320, 0], [0, 0, 0.690278]])
    initial_position = np.r_[0, 0, 0]
    my_sensor = WatchIMU(in_file='data/gyro_calibration.csv', R_init=initial_orientation, pos_init=initial_position)
    my_sensor.set_qtype("analytical")


    my_sensor._calc_orientation()
    print(my_sensor.omega)
    my_sensor.calc_position()
    print(my_sensor.pos[0])
    print(my_sensor.pos[-1])
    xs = my_sensor.pos[:,0]
    ys = my_sensor.pos[:,1]
    zs = my_sensor.pos[:,2]

    xbeg = my_sensor.pos[:,0][0]
    ybeg = my_sensor.pos[:,1][0]
    zbeg = my_sensor.pos[:,2][0]
    xfin = my_sensor.pos[:,0][-1]
    yfin = my_sensor.pos[:,1][-1]
    zfin = my_sensor.pos[:,2][-1]

    fig = plt.figure(1)
    plt.plot(xs)
    plt.plot(ys)
    plt.plot(zs)

    
    fig = plt.figure(2)
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel("X")
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.scatter3D(xs, ys, zs)
    ax.scatter3D(xbeg, ybeg, zbeg)
    ax.scatter3D(xfin, yfin, zfin)

      
    plt.show()
    print('Done')