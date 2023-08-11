from robotic_arm import RobotArm
from collect_data import MyWatch
import asyncio

async def dummy_robot(queue):
    item = await queue.get()
    print(item)

async def main():
    queue = asyncio.Queue()
    #arm = RobotArm()
    watch = MyWatch(queue)

    get_data = asyncio.create_task(watch.run())
    #do_action = asyncio.create_task(arm.run(queue))
    dummy_do = asyncio.create_task(dummy_robot(queue))

    await asyncio.gather(get_data, dummy_do)

if __name__ == '__main__':
    asyncio.run(main())
    print('done')