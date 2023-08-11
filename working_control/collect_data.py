from skinematics import quat
from skinematics.imus import IMU_Base
from touch_sdk import Watch
from touch_sdk.watch import SensorFrame, Hand
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import asyncio
import argparse
import json

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.components.board import Board
from viam.components.arm import Arm
from viam.services.motion import MotionClient
from viam.proto.common import Pose, PoseInFrame, Vector3, Geometry, GeometriesInFrame, RectangularPrism, WorldState
from viam.proto.service.motion import Constraints, LinearConstraint, OrientationConstraint

class MyWatch(Watch):
    def __init__(self, queue):
        super().__init__()
        self.file_path = 'data/watch_data.csv'
        self.x = 0
        self.y = 0
        self.z = 0
        self.data_index = 0
        self.latest_x = []
        self.latest_y = []

        # create/empty file to save data to
        f = open(self.file_path, 'w')  # create new file or overwrite existing file
        f.close()

        self.queue = queue

        plt.ion()
        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X Label')
        self.ax.set_ylabel('Y Label')
        self.ax.set_zlabel('Z Label')

    async def update_plot(self):
        await self.queue.put("updating plot")
        # Clear the existing plot and create a new one
        self.ax.clear()
        self.ax.scatter(self.x, self.y, self.z, s=50)  
        self.ax.set_xlim([-200, 200])
        self.ax.set_ylim([-200, 200])
        self.ax.set_zlim([-200, 200])
        plt.title("Real-time Plot")
        plt.draw()
        plt.pause(0.05)        
        
    async def _proto_on_sensors(self, frames, timestamp):
        frame = frames[-1]
        sensor_frame = SensorFrame(
            acceleration=_protovec3_to_tuple(frame.acc),
            gravity=_protovec3_to_tuple(frame.grav),
            angular_velocity=_protovec3_to_tuple(frame.gyro),
            orientation=_protoquat_to_tuple(frame.quat),
            magnetic_field=(
                _protovec3_to_tuple(frame.mag) if frame.HasField("mag") else None
            ),
            magnetic_field_calibration=(
                _protovec3_to_tuple(frame.magCal)
                if frame.HasField("magCal")
                else None
            ),
            timestamp=timestamp,
        )
        self.on_sensors(sensor_frame)
        await self._on_arm_direction_change(sensor_frame)
    async def _on_arm_direction_change(self, sensor_frame: SensorFrame):
        def normalize(vector):
            length = sum(x * x for x in vector) ** 0.5
            return [x / length for x in vector]

        grav = normalize(sensor_frame.gravity)

        av_x = -sensor_frame.angular_velocity[2]  # right = +
        av_z = -sensor_frame.angular_velocity[1]  # down = +
        av_y = -sensor_frame.angular_velocity[0]  # forward = +

        handedness_scale = -1 if self.hand == Hand.LEFT else 1

        x_scale = 0.5
        y_scale = 1.5
        z_scale = 1

        delta_x = (av_x * grav[2] + av_z * grav[1]) * x_scale
        delta_z = (-1 * handedness_scale * (av_z * grav[2] - av_x * grav[1])) * z_scale
        delta_y = (-1 * handedness_scale * (av_y * grav[2] - av_z * grav[0])) * y_scale

        await self.on_arm_direction_change(delta_x, delta_y, delta_z)

    async def on_arm_direction_change(self, delta_x, delta_y, delta_z):
        
        speed = 2

        # Integrate
        self.x += delta_x * speed
        self.y += delta_y * speed
        self.z += delta_z * speed

        self.data_index += 1
        if self.data_index == 100:
            await self.update_plot()
            self.data_index = 0
    
        plt.title("Live Updating 3D Graph")

    
    def on_tap(self):
        self.trigger_haptics(1.0, 20)

    def on_touch_down(self, x, y):
        # reset to home position
        self.x = 0
        self.y = 0
        self.z = 0
        







