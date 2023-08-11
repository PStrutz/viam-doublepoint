import asyncio


from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.components.camera import Camera, Arm
from touch_sdk import Watch


class MyWatch(Watch):
    def on_tap(self):
        print('Tap')

watch = MyWatch()
watch.start()

async def connect():
    creds = Credentials(
        type='robot-location-secret',
        payload='xgq8apgpcquclbi5igxbuo6zvm45o8qvhrpb6wlh701e3p1x')
    opts = RobotClient.Options(
        refresh_interval=0,
        dial_options=DialOptions(credentials=creds)
    )
    return await RobotClient.at_address('20--project-main.8uk9zc4kv5.viam.cloud', opts)

async def main():
    robot = await connect()

    print('Resources:')
    print(robot.resource_names)

    # arm
    arm = Arm.from_robot(robot, "arm")
    arm_return_value = await arm.get_end_position()
    print(f"arm get_end_position return value: {arm_return_value}")

    

    # Don't forget to close the robot when you're done!
    await robot.close()

if __name__ == '__main__':
    asyncio.run(main())

