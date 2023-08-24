from dataclasses import dataclass
from enum import Enum
import asyncio
from typing import Tuple, Optional
import asyncio_atexit

from touch_sdk.uuids import PROTOBUF_OUTPUT, PROTOBUF_INPUT
from touch_sdk.utils import unpack_chained
from touch_sdk.watch_connector import WatchConnector

# pylint: disable=no-name-in-module
from touch_sdk.protobuf.watch_output_pb2 import Update, Gesture, TouchEvent
from touch_sdk.protobuf.watch_input_pb2 import InputUpdate, HapticEvent

from robotic_arm import ViamRobot
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions

# import matplotlib.pyplot as plt


__doc__ = """Discovering Touch SDK compatible BLE devices and interfacing with them."""


@dataclass(frozen=True)
class SensorFrame:
    """A Frozen container class for values of all streamable Touch SDK sensors."""

    acceleration: Tuple[float]
    gravity: Tuple[float]
    angular_velocity: Tuple[float]
    orientation: Tuple[float]
    magnetic_field: Optional[Tuple[float]]
    magnetic_field_calibration: Optional[Tuple[float]]
    timestamp: int


class Hand(Enum):
    """Which hand the watch is worn on."""

    NONE = 0
    RIGHT = 1
    LEFT = 2


def _protovec2_to_tuple(vec):
    return (vec.x, vec.y)


def _protovec3_to_tuple(vec):
    return (vec.x, vec.y, vec.z)


def _protoquat_to_tuple(vec):
    return (vec.x, vec.y, vec.z, vec.w)


class Watch:
    """Scans Touch SDK compatible Bluetooth LE devices and connects to the first one
    of them that approves the connection.

    Watch also parses the data that comes over Bluetooth and returns it through
    callback methods."""

    def __init__(self, name_filter=None, connect_viam=True):
        """Creates a new instance of Watch. Does not start scanning for Bluetooth
        devices. Use Watch.start to enter the scanning and connection event loop.

        Optional name_filter connects only to watches with that name (case insensitive)"""
        self._connector = WatchConnector(
            self._on_approved_connection, self._on_protobuf, name_filter
        )
        
        self._viam_robot_client = None
        self._connect_viam = connect_viam
        self._info_queue = asyncio.Queue()
        self._viam_robot = None
        
        self._client = None
        self._stop_event = None

        self._event_loop = None
        self._tasks = []
        self._robot_task = None

        self.custom_data = None
        if hasattr(self.__class__, "custom_data"):
            self.custom_data = self.__class__.custom_data

        self.hand = Hand.NONE

        # additions:
        self.file_path = 'data/watch_data.csv'
        self.pos = [390, 105, 500]
        self.data_index = 0
        self.latest_x = []
        self.latest_y = []

        # create/empty file to save data to
        f = open(self.file_path, 'w')  # create new file or overwrite existing file
        f.close()


    def start(self):
        """Blocking event loop that starts the Bluetooth scanner

        More handy than Watch.run when only this event loop is needed."""
        try:
            asyncio.run(self.run())
        except KeyboardInterrupt:
            pass

    def stop(self):
        """Stop the watch, disconnecting any connected devices."""
        self._stop_event.set()

    async def run(self):
        """Asynchronous blocking event loop that starts the Bluetooth scanner.

        Makes it possible to run multiple async event loops with e.g. asyncio.gather."""

        self._event_loop = asyncio.get_running_loop()
        self._stop_event = asyncio.Event()

        asyncio_atexit.register(self.stop)

        # connect to the robot
        if self._connect_viam is True:
            self._viam_robot = ViamRobot()
            self._viam_robot_client = await self._viam_robot.connect()
            print("connected to robot")
            # self._viam_robot_client = "robot!"
            
        # connect to the watch
        await self._connector.start()
        await self._stop_event.wait()
        await self._connector.stop()
        if self._viam_robot is not None:
            await self._viam_robot_client.close()

    async def _on_approved_connection(self, client):
        self._client = client

        await self._fetch_info(client)
        await self._subscribe_to_custom_characteristics(client)

    async def _fetch_info(self, client):
        data = await client.read_gatt_char(PROTOBUF_OUTPUT)
        update = Update()
        update.ParseFromString(bytes(data))
        if update.HasField("info"):
            self.hand = Hand(update.info.hand)

    async def _connect_viam_receiver(self, secret, address):
        creds = Credentials(
            type='robot-location-secret',
            payload=secret)
        opts = RobotClient.Options(
            refresh_interval=0,
            dial_options=DialOptions(credentials=creds)
        )
        return await RobotClient.at_address(address, opts)


    # Custom characteristics

    async def _subscribe_to_custom_characteristics(self, client):
        if self.custom_data is None:
            return

        subscriptions = [
            client.start_notify(uuid, self._on_custom_data) for uuid in self.custom_data
        ]
        await asyncio.gather(*subscriptions)

    async def _on_custom_data(self, characteristic, data):
        format_string = self.custom_data.get(characteristic.uuid)

        if format_string is None:
            return

        content = unpack_chained(format_string, data)

        await self.on_custom_data(characteristic.uuid, content)

    async def on_custom_data(self, uuid: str, content: Tuple):
        """Receive data from custom characteristics"""

    # Main protobuf characteristic

    async def _on_protobuf(self, message):
        await self._proto_on_sensors(message.sensorFrames, message.unixTime)
        await self._proto_on_gestures(message.gestures)
        await self._proto_on_touch_events(message.touchEvents)
        await self._proto_on_button_events(message.buttonEvents)
        await self._proto_on_rotary_events(message.rotaryEvents)

        if message.HasField("info"):
            await self._proto_on_info(message.info)

        if message.pressure != 0.0:
            await self.on_pressure(message.pressure)

    # Sensor events

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
        await self.on_sensors(sensor_frame)
        await self._on_arm_direction_change(sensor_frame)

    async def on_sensors(self, sensor_frame: SensorFrame):
        """Callback when accelerometer, gyroscope, gravity, orientation, and
        magnetic field are changed. Guaranteed to have values for everything but
        magnetic field information in every update."""

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

    async def on_arm_direction_change(self, delta_x: float, delta_y: float, delta_z: float):
        """Gyroscope-based raycasting output. Called after sensor updates."""
        speed = 3

        # Integrate
        self.pos[0] += delta_x * speed
        self.pos[1] += delta_y * speed
        self.pos[2] += delta_z * speed * 2

        self.data_index += 1
        if self.data_index == 500:
            print("sending to robot")
            # await self._info_queue.put(["move_to", self.pos])
            self.data_index = 0
            # await self._robot_task
            await self._send_to_robot(self._viam_robot_client, ["move_to", self.pos])

    async def on_pressure(self, pressure: float):
        """Called when new pressure value (in hectopascals) is received."""

    # Gestures

    async def _proto_on_gestures(self, gestures):
        if any(g.type == Gesture.GestureType.TAP for g in gestures):
            await self.on_tap()

    async def on_tap(self):
        """Called when the tap gesture happens."""
        print("got tap")
        await self._send_to_robot(self._viam_robot_client, ["grab", None])


    # Touch screen

    async def _proto_on_touch_events(self, touch_events):
        for touch in touch_events:
            coords = _protovec2_to_tuple(touch.coords[0])
            if touch.eventType == TouchEvent.TouchEventType.BEGIN:
                await self.on_touch_down(*coords)
            elif touch.eventType == TouchEvent.TouchEventType.END:
                await self.on_touch_up(*coords)
            elif touch.eventType == TouchEvent.TouchEventType.MOVE:
                await self.on_touch_move(*coords)
            elif touch.eventType == TouchEvent.TouchEventType.CANCEL:
                await self.on_touch_cancel(*coords)

    async def on_touch_down(self, x: float, y: float):
        # reset to home position
        self.pos = [390, 105, 500]
        await self._send_to_robot(self._viam_robot_client, ["reset", None])


    async def on_touch_up(self, x: float, y: float):
        """Touch screen touch ends."""

    async def on_touch_move(self, x: float, y: float):
        """Touch screen touch moves."""

    async def on_touch_cancel(self, x: float, y: float):
        """Touch screen touch becomes a swipe gesture that goes to another view."""

    # Button

    async def _proto_on_button_events(self, buttons):
        if any(b.id == 0 for b in buttons):
            await self.on_back_button()

    async def on_back_button(self):
        """Back button of the watch is pressed and released.

        Wear OS does not support separate button down and button up events."""

    # Rotary

    async def _proto_on_rotary_events(self, rotary_events):
        for rotary in rotary_events:
            await self.on_rotary(-rotary.step)

    async def on_rotary(self, direction: int):
        """Rotary dial around the watch screen is turned.

        direction: +1 for clockwise, -1 for counterclockwise."""

    # Info

    async def _proto_on_info(self, info):
        self.hand = Hand(info.hand)

    # Haptics

    async def trigger_haptics(self, intensity: float, duration_ms: int):
        """Trigger vibration haptics on the watch.

        intensity: between 0 and 1
        duration_ms: between 0 and 5000"""
        input_update = await self._create_haptics_update(intensity, duration_ms)
        await self._write_input_characteristic(input_update.SerializeToString(), self._client)

    @staticmethod
    def _create_haptics_update(intensity, length):
        clamped_intensity = min(max(intensity, 0.0), 1.0)
        clamped_length = min(max(int(length), 0), 5000)
        haptic_event = HapticEvent()
        haptic_event.type = HapticEvent.HapticType.ONESHOT
        haptic_event.length = clamped_length
        haptic_event.intensity = clamped_intensity
        input_update = InputUpdate()
        input_update.hapticEvent.CopyFrom(haptic_event)
        return input_update

    async def _write_input_characteristic(self, data, client):
        if self._event_loop is not None:
            self._tasks.append(self._event_loop.create_task(
                self._async_write_input_characteristic(PROTOBUF_INPUT, data, client)
            ))

    async def _async_write_input_characteristic(self, characteristic, data, client):
        if client:
            await client.write_gatt_char(characteristic, data, True)

    async def _send_to_robot(self, robot, data):
        if self._event_loop is not None and self._viam_robot is not None:
            self._tasks.append(self._event_loop.create_task(
                self._async_send_to_robot(robot, data)
            ))

    async def _async_send_to_robot(self, robot, data):
        await self._viam_robot.run(robot, data)
