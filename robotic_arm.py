import asyncio
import argparse
import json
import time

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.components.board import Board
from viam.components.arm import Arm
from viam.services.motion import MotionClient
from viam.proto.common import Pose, PoseInFrame, Vector3, Geometry, GeometriesInFrame, RectangularPrism, WorldState
from viam.proto.service.motion import Constraints, LinearConstraint, OrientationConstraint


class ViamRobot():

    def __init__(self):
        # The amount to move in mm for each command forward, backward, left, right
        self.move_increment = 50

        # Define home position to return to 
        self.home_plane = 500.0
        self.home_pose = Pose(x=390.0, y=105.0, z=self.home_plane, o_x=0, o_y=0, o_z=-1, theta=0)

        # Define plane to grab on
        self.grab_plane = 240.0
        self.grab_closed = True

        self.constraints = Constraints(orientation_constraint = [OrientationConstraint()])

        self.world_state = self.get_world_state()


    # Define world_state representing the physical environment the claw exists in 
    def get_world_state(self):
        with open('working_control/obstacles.json', 'r') as f:
            geometries = json.load(f)

        world_state_obstacles = []
        # for geometry in geometries:
        #     center = geometry['translation']
        #     orientation = geometry['orientation']['value']
        #     center = Pose(
        #         x=center['x'], 
        #         y=center['y'], 
        #         z=center['z'], 
        #         o_x=orientation['x'], 
        #         o_y=orientation['y'], 
        #         o_z=orientation['z'], 
        #         theta=orientation['th'],
        #     )
        #     dims = Vector3(x=geometry['x'], y=geometry['y'], z=geometry['z'])
        #     world_state_obstacles.append(Geometry(center=center, box=RectangularPrism(dims_mm=dims), label=geometry['label']))

        obstacles_in_frame = GeometriesInFrame(reference_frame="world", geometries=world_state_obstacles)
        return WorldState(obstacles=[obstacles_in_frame])
    

    async def connect(self):
        creds = Credentials(
            type='robot-location-secret',
            payload='hvzgmm622vidvvg6meh6ktgsplg4dnehqgjeseybkglkt3wd')
        opts = RobotClient.Options(
            refresh_interval=0,
            dial_options=DialOptions(credentials=creds)
        )
        return await RobotClient.at_address('office-claw-main.x009joptm1.viam.cloud', opts)

    async def grab(self, board, doGrab):
        # Note that the pin supplied is a placeholder. Please change this to a valid pin you are using.
        pin = await board.gpio_pin_by_name('8')
        if doGrab == True:
            #opens the gripper/release
            await pin.set(True)
        else:
            #closes the gripper/grab
            await pin.set(False)

    async def move_absolute(self, arm, motion_service, pose):
        destination = PoseInFrame(reference_frame="world", pose=pose)
        print("am moving")
        await motion_service.move(component_name=arm, destination=destination, world_state=self.world_state, constraints=self.constraints)

    async def home(self, arm, motion_service):
        # Makes sure to first move the arm up in z axis
        await self.move_z(arm, motion_service, 500)

        # Generate a sample "home" pose around the drop hole and demonstrate motion
        home_pose_in_frame = PoseInFrame(reference_frame="world", pose=self.home_pose)

        await motion_service.move(component_name=arm, destination=home_pose_in_frame, world_state=self.world_state, constraints=self.constraints)
        time.sleep(5)

    async def move_to_offset(self, arm, motion_service, offset):
        # Get current position of the arm
        current_position = await motion_service.get_pose(component_name=arm, destination_frame = "", supplemental_transforms = None)
        print('current position: ', current_position)
        
        # Calculate new pose to move the arm to
        pose = Pose(
            x=current_position.pose.x + offset.x, 
            y=current_position.pose.y + offset.y,
            z=current_position.pose.z + offset.z,
            o_x=0, 
            o_y=0, 
            o_z=-1, # negative z means claw will point down
            theta=0
        )
        print('moving to position: ', pose)

        # Move arm
        destination = PoseInFrame(reference_frame="world", pose=pose)
        await motion_service.move(component_name=arm, destination=destination, world_state=self.world_state, constraints=self.constraints)


    async def move_z(self, arm, motion_service, z):
        # Get current position of the arm
        current_position = await motion_service.get_pose(component_name=arm, destination_frame = "", supplemental_transforms = None)
        print('current_position: ', current_position)
        
        # Construct new pose to get to desired z position
        pose = Pose(
            x=current_position.pose.x, 
            y=current_position.pose.y, 
            z = z,
            o_x= 0, 
            o_y=0, 
            o_z=-1, # negative z means claw will point down
            theta=0
        )
        print('moving to position: ', pose)

        # Move arm
        destination = PoseInFrame(reference_frame="world", pose=pose)
        await motion_service.move(component_name=arm, destination=destination, world_state=self.world_state, constraints=self.constraints)

    
    async def run(self, robot, data):
        print("got data ", data)
        robot = await self.connect()

        # Pose using motion service, grabbing the service from local computer
        motion_service = MotionClient.from_robot(robot, "planning:builtin")
        
        # myBoard
        my_board = Board.from_robot(robot, "myBoard")
        # my Subpart name, arm
        my_arm_resource= Arm.get_resource_name("planning:myArm")
        my_arm_resource.name= "myArm"

            
        
        [command, action] = data
        if command == "move_to":
            
            if action is None or len(action) != 3:
                print("not supposed to be here")
                return
            if action[0] > 500:
                action[0] == 500
            elif action[0] < 0:
                action[0] = 0
            if action[1] > 500:
                action[1] == 500
            elif action[1] < -500:
                action[1] = -500
            if action[2] > 700:
                action[2] = 700
            elif action[2] < 200:
                action[2] = 200
            print(command, action)
            print("moving")
           
            await self.move_absolute(my_arm_resource, motion_service, Pose(x=action[0], y=action[1], z=action[2], o_x=0, o_y=0, o_z=-1, theta=0))
        if command == "grab":
            print("using claw")
            self.grab_closed = not self.grab_closed
            await self.grab(my_board, self.grab_closed)
        if command == "reset":
            print("resetting")
            await self.home(my_arm_resource, motion_service) 

        # # Don't forget to close the robot when you're done!
        await robot.close()



