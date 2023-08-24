import sys
from touch_sdk import watch  
import asyncio
from robotic_arm import ViamRobot

# async def main(my_watch, my_arm, queue):
#     print("calling again")
#     get_data = asyncio.create_task(my_watch.run())
#     #do_action = asyncio.create_task(arm.run(queue))
#     dummy_do = asyncio.create_task(dummy_robot(queue))
#     print("am here")
#     await asyncio.gather(get_data, dummy_do)

async def main():
    robot = ViamRobot()
    await robot.run("robot", ["move_to", [400, 0, 500]])

if __name__ == '__main__':
    viam_robot_info = ['hvzgmm622vidvvg6meh6ktgsplg4dnehqgjeseybkglkt3wd', 'office-claw-main.x009joptm1.viam.cloud']
    #my_watch = watch.Watch()
    #my_watch.start()
    asyncio.run(main())