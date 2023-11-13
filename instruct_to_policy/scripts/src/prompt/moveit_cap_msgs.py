message_tabletop_ui = [
{
"role":"system",
"content": """
You are a strict assistant, translating natural language to a python script to control a robotic arm.
Your output should be a python script that can be executed to perform the task described in the input.
Your reasoning process and detailed planning can be written as python comment in the scripts.
You can use the following imported modules and functions in your script:

'''
# Import rospy, the foundational Python library for ROS (Robot Operating System)
import rospy

# Import move_group, the class handle for moving the robot arm in MoveIt!
import move_group

# Import various ROS message types for handling robot pose and orientation
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# Import parse_question, this function queries a question to a multi-modal model and retrieves the text answer
from utils import parse_question              

# Import utility functions for perception
from perception_utils import (
    get_object_center_position,  # Returns the position of an object in the world frame. Returns: position: np.array [x,y,z]
    get_3d_bbox,                 # Returns the 3D bounding box of an object in the world frame. Args: object_name: str. Returns: bbox: np.array [x_min, y_min, z_min, x_max, y_max, z_max]
    get_obj_name_list,           # Returns a list of names of objects present in the scene
    parse_grasp_pose,            # Predict a grasp pose for a specified object. Args: object_name: str, preferred_position: Optional(np.array), preferred_direction: Optional(np.array). Returns: grasp_pose: Pose
    parse_place_pose,            # Predict the place pose for an object relative to a receptacle. Args: object_name: str, receptacle_name: Optional(str), position: Optional(np.array) [x,y,z], . Returns: place_pose: Pose
    detect_objects               # Detects objects and update objects states after robot action execution and returns their names as a list.
)

# Import utility functions for robot motion planning and execution
from motion_utils import (
    attach_object,  # Attaches an object to the robot gripper in the planning space. Call this function right after closing the gripper
    detach_object   # Detaches an object from the robot gripper in the planning space. Call this function right after opening the gripper.
    open_gripper    # Open the gripper 
    close_gripper   # Close the gripper
    move_to_pose    # Move the gripper to pose.
    get_gripper_pose # Get the gripper pose
    grasp           # Executes a grasp motion at the grasp_pose. Args: grasp_pose: Pose
)
'''

Your generated content should only contain comments starting with '#' and python code!
"""
},
{# Open a Drawer
"role":"user",
"content": "objects = ['drawer1', 'table', 'plants'] \n # open the drawer"
},
{
"role":"assistant",
"content":
'''
# Detailed planning: 
# Step 1: Grasp the drawer handle 
# Step 2: Pull the drawer handle, move in +x direction 
# Step 3: Release the gripper from drawer handle 


# Grasp the drawer handle
open_gripper()
grasp_pose = parse_grasp_pose('drawer1_handle')
grasp(grasp_pose)
close_gripper()
attach_object('drawer1_handle')

# Pull the drawer handle, move in +x direction
direction = [1, 0, 0] # (x, y, z)
move_in_direction(direction, distance=0.2)

# Release the drawer
open_gripper()
detach_object('drawer1_handle')
'''
},
{# Close a Drawer
"role":"user",
"content": "objects = ['drawer1', 'table', 'plants'] \n # close the drawer"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Grasp the drawer handle
# Step 2: Push the drawer handle, move in -x direction
# Step 3: Release the gripper from drawer handle


# Grasp the drawer handle
open_gripper()
grasp_pose = parse_grasp_pose('drawer1_handle')
grasp(grasp_pose)
close_gripper()
attach_object('drawer1_handle')

# Push the drawer handle, move in -x direction
direction = [-1, 0, 0] # (x, y, z)
move_in_direction(direction, distance=0.2)

# Release the drawer
open_gripper()
detach_object('drawer1_handle')
'''
},
{# Put object into the drawer
"role":"user",
"content": "objects = ['drawer1', 'drawer2', 'table', 'red_box'] \n # put the red box into the top drawer"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Grasp the handle of the top drawer
# Step 2: Pull the handle of the top drawer, move in +x direction
# Step 3: Release the handle of the top drawer 
# Step 4: Grasp the red box
# Step 5: Move the red box into the drawer
# Step 6: Release the red box

# Grasp the handle of the top drawer
open_gripper()
grasp_pose = parse_grasp_pose('drawer1_handle')
grasp(grasp_pose)
close_gripper()
attach_object('drawer1_handle')

# Pull the handle of the top drawer, move in +x direction
direction = [1, 0, 0] # (x, y, z)
move_in_direction(direction, distance=0.2)

# Release the handle of the top drawer
open_gripper()
detach_object('drawer1_handle')

# Grasp the red box
grasp_pose = parse_grasp_pose('red_box')
grasp(grasp_pose)
close_gripper()
attach_object('red_box')

# Move the red box into the drawer
place_pose = parse_place_pose('red_box', 'drawer1')
move_to_pose(place_pose)

# Release the red box
open_gripper()
detach_object('red_box')
'''
},
{# Pick object from the drawer
"role":"user",
"content": "objects = ['drawer0', 'drawer1', 'table', 'knife', 'plate'] \n # pick the knife from the top drawer and put it on the plate"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Grasp the handle of the top drawer
# Step 2: Pull the handle of the top drawer, move in +x direction
# Step 3: Release the handle of the top drawer
# Step 4: Grasp the knife
# Step 5: Move the knife to the plate
# Step 6: Release the knife

 
# Grasp the handle of the top drawer
open_gripper()
grasp_pose = parse_grasp_pose('drawer0_handle')
grasp(grasp_pose)
close_gripper()
attach_object('drawer0_handle')

# Pull the handle of the top drawer, move in +x direction
direction = [1, 0, 0] # (x, y, z)
move_in_direction(direction, distance=0.2)
 
# Release the handle of the top drawer
open_gripper()
detach_object('drawer0_handle')

# Grasp the knife
grasp_pose = parse_grasp_pose('knife')
grasp(grasp_pose)
close_gripper()
attach_object('knife')

# Move the knife to the plate
place_pose = parse_place_pose('knife', 'plate')
move_to_pose(place_pose)

# Release the knife
open_gripper()
detach_object('knife')
'''  
},
{# Turn a Faucet Clockwise
"role":"user",
"content": "objects = ['faucet', 'handle', 'sink'] \n # turn the faucet clockwise"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Grasp the faucet handle
# Step 2: Generate an arc path around the faucet
# Step 3: Turn the handle clockwise
# Step 4: Release the faucet handle

# Grasp the faucet handle
grasp_pose = parse_grasp_pose(object='faucet_handle')
open_gripper()
grasp(grasp_pose)
close_gripper()
attach_object('faucet_handle')

# Generate an arc path around the faucet
current_pose = get_gripper_pose()
waypoints = generate_arc_path(current_pose, center=get_object_center_position('faucet'), radius=0.1, n=20)

# Turn the handle clockwise
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)

# Release the faucet handle
open_gripper()
detach_object('faucet_handle')
'''
},
{# Open a Door 
"role":"user",
"content": "objects = ['door'] \n # open the door"
},
{
"role":"assistant",
"content":
'''
# Detailed planning: 
# Step 1: Grasp the door handle 
# Step 2: Pull the door handle downwards with an arc path of small radius by 30 degrees
# Step 3: pull the door horizontally with an arc path of large radius by 60 degrees
# Step 4: release the door handle 

# Grasp the door handle
grasp_pose = parse_grasp_pose('door_handle')
grasp(grasp_pose)
close_gripper()
attach_object('door_handle')

# Pull the door handle downwards with an arc path of small radius by 30 degrees
current_pose = get_gripper_pose()
waypoints = generate_arc_path(current_pose, center=get_object_center_position('door_handle'), radius=0.1, n=20, angle=30)
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)

# pull the door horizontally with an arc path of large radius by 60 degrees
current_pose = get_gripper_pose()
waypoints = generate_arc_path(current_pose, center=get_object_center_position('door_handle'), radius=0.5, n=40, angle=60)
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)

# Release the door handle
open_gripper()
detach_object('door_handle')
'''
},
{# Pouring Liquid
"role":"user",
"content": "objects = ['bottle', 'glass', 'table'] \n # grasp a bottle and pour the liquid into the glass, then place it back on the table"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Grasp the bottle at the neck
# Step 2: Move the bottle to above the glass
# Step 3: Rotate the bottle to pour the liquid into the glass
# Step 4: Rotate the bottle back to the original orientation
# Step 5: Place the bottle back on the table

# Grasp the bottle at the neck
open_gripper()
grasp_pose = parse_grasp_pose(object_name='bottle', description="bottle neck")
grasp(grasp_pose)
close_gripper()
attach_object('bottle')

# Move the bottle to above the glass
pour_pose = parse_place_pose(object_name='bottle', receptacle_name='glass')
move_to_pose(pour_pose)

# Rotate the bottle to pour the liquid into the glass
pour_angle = 90
current_pose = get_gripper_pose()
waypoints = generate_arc_path(current_pose, center=get_object_center_position('bottle'), radius=0.1, n=30, angle=pour_angle)
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)

# Rotate the bottle back to the original orientation
current_pose = get_gripper_pose()
waypoints = generate_arc_path(current_pose, center=get_object_center_position('bottle'), radius=0.1, n=30, angle=-pour_angle)
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)

# Place the bottle back on the table
place_pose = parse_place_pose(object_name='bottle', receptacle_name='table')
move_to_pose(place_pose)
open_gripper()
detach_object('bottle')
'''
},
{# Mixing Ingredients
"role":"user",
"content": "objects = ['spoon', 'bowl', 'table'] \n # use the spoon to mix the ingredients in the bowl"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Grasp the spoon
# Step 2: Define the spoon start pose in the bowl for mixing ingredients
# Step 3: Move the spoon into the start pose 
# Step 4: Stir the ingredients in the bowl by moving the spoon horizontally in a random directions
# Step 5: Release the spoon

# Grasp the spoon
open_gripper()
grasp_pose = parse_grasp_pose(object_name='spoon')
grasp(grasp_pose)
close_gripper()
attach_object('spoon')

# Define the spoon start pose in the bowl for mixing ingredients
spoon_start_pose = parse_place_pose(object_name='spoon', receptacle_name='bowl')
 
# Move the spoon into the start pose
move_to_pose(spoon_start_pose)
 
# Stir the ingredients in the bowl by moving the spoon horizontally in a random direction
for i in range(5):
    direction = np.random.rand(3) 
    direction[2] = 0 # only move in x-y plane
    direction = direction / np.linalg.norm(direction)
    move_in_direction(direction, distance=0.05)
    
# Release the spoon
open_gripper()
detach_object('spoon')
'''
},
{# Retrieve an item from a high shelf
"role":"user",
"content": "objects = ['shelf', 'bowl', 'table'] \n # retrieve a bowl from the high shelf and put it on the table"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Grasp the bowl 
# Step 2: Move the bowl to the table
# Step 3: Release the bowl

# Grasp the bowl
open_gripper()
grasp_bowl_pose = parse_grasp_pose(object='bowl', description='a bowl from the high shelf')
grasp(grasp_bowl_pose)
close_gripper()
attach_object('bowl')

# Move the bowl to the table
put_bowl_pose = parse_place_pose(object_name='bowl', receptable_name='table')
move_to_pose(put_bowl_pose)

# Release the bowl
open_gripper()
detach_object('bowl')
'''
},
{# Move an object away from another object
"role":"user",
"content": "objects = ['bowl', 'table', 'apple'] \n # move the apple away from the bowl by 0.1m"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Grasp the apple
# Step 2: Calculate the position to move the apple to
# Step 3: Move the apple to the calculated position
# Step 4: Release the apple

# Grasp the apple
open_gripper()
grasp_apple_pose = parse_grasp_pose(object='apple')
grasp(grasp_apple_pose)
close_gripper()
attach_object('apple')

# Calculate the position to move the apple to
apple_position = get_object_center_position('apple')
bowl_position = get_object_center_position('bowl')
direction = apple_position - bowl_position
direction = direction / np.linalg.norm(direction)
move_position = apple_position + direction * 0.1

# Move the apple to the calculated position
put_apple_pose = parse_place_pose(object_name='apple', position=move_position)
move_to_pose(put_apple_pose)
 
# Release the apple
open_gripper()
detach_object('apple')
'''
},
{# Swap objects in the two containers
"role":"user",
"content": "objects = ['plate', 'fry_pan', 'table', 'peach', 'apple'] \n # swap the positions of the peach in the plate and the apple in the fry pan"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Grasp the peach in the plate
# Step 2: Move the peach onto the table
# Step 3: Release the peach 
# Step 4: Wait for environment to be static and detect objects new states 
# Step 5: Grasp the apple in the fry pan
# Step 6: Move the apple into the plate
# Step 7: Release the apple
# Step 8: Wait for environment to be static and detect objects new states
# Step 9: Grasp the peach on the table
# Step 10: Move the peach into the fry pan

# Grasp the peach in the plate
peach_grasp_pose = parse_grasp_pose(object_name='peach')
open_gripper()
grasp(peach_grasp_pose)
close_gripper()
attach_object('peach')

# Move the peach onto the table
peach_place_pose = parse_place_pose(object_name='peach', receptable_name='table')
move_to_pose(peach_place_pose)

# Release the peach
open_gripper()
detach_object('peach')

# Wait for environment to be static and detect objects new states
rospy.sleep(5)
detect_objects()

# Grasp the apple in the fry pan
apple_grasp_pose = parse_grasp_pose(object_name='apple')
grasp(apple_grasp_pose)
close_gripper()
attach_object('apple')

# Move the apple into the plate
apple_place_pose = parse_place_pose(object_name='apple', receptable_name='plate')
move_to_pose(apple_place_pose)

# Release the apple
open_gripper()
detach_object('apple')

# Wait for environment to be static and detect objects new states
rospy.sleep(5)
detect_objects()
 
# Grasp the peach on the table
peach_grasp_pose = parse_grasp_pose(object_name='peach')
grasp(peach_grasp_pose)
close_gripper()
attach_object('peach')

# Move the peach into the fry pan
peach_place_pose = parse_place_pose(object_name='peach', receptable_name='fry_pan')
move_to_pose(peach_place_pose)
 
# Release the peach
open_gripper()
detach_object('peach')
'''
},
]

message_parse_obj_name = [
{
"role":"system",
"content": '''
You are a strict assistant, translating natural language to a python script to parse the objects indicated by instructions.
Your output should be a python script that can be executed to perform the task described in the input.
'''
},
{
"role": "assistant",
"content": '''
import numpy as np
from env_utils import get_object_center_position, parse_pose
from utils import get_object_center_positionitions_np
'''
},
{
"role": "user",
"content": "objects = ['blue block', 'cyan block', 'purple bowl', 'gray bowl', 'brown bowl', 'pink block', 'purple block'] \n # the block closest to the purple bowl"
},
{
"role": "assistant",
"content": '''
block_names = ['blue block', 'cyan block', 'purple block']
block_positions = get_object_center_positionitions_np(block_names)
closest_block_idx = np.argsort(np.linalg.norm(block_positions - get_object_center_position('purple bowl'), axis=1))[0]
closest_block_name = block_names[closest_block_idx]
ret_val = closest_block_name
'''
},
{
"role": "user",
"content": "objects = ['brown bowl', 'banana', 'brown block', 'apple', 'blue bowl', 'blue block'] \n # the blocks"
},
{
"role": "assistant",
"content": '''
ret_val = ['brown block', 'blue block']
'''
},
{
"role": "user",
"content": "objects = ['brown bowl', 'banana', 'brown block', 'apple', 'blue bowl', 'blue block'] \n # the brown objects"
},
{
"role": "assistant",
"content": '''
ret_val = ['brown bowl', 'brown block']
'''
},
{
"role": "user",
"content": "objects = ['brown bowl', 'banana', 'brown block', 'apple', 'blue bowl', 'blue block'] \n # a fruit that's not the apple"
},
{
"role": "assistant",
"content": '''
fruit_names = ['banana', 'apple']
for fruit_name in fruit_names:
    if fruit_name != 'apple':
        ret_val = fruit_name
'''
},
{
"role": "user",
"content": "objects = ['blue block', 'cyan block', 'purple bowl', 'brown bowl', 'purple block'] \n # blocks above the brown bowl"
},
{
"role": "assistant",
"content": '''
block_names = ['blue block', 'cyan block', 'purple block']
brown_bowl_pos = get_object_center_position('brown bowl')
use_block_names = []
for block_name in block_names:
    if get_object_center_position(block_name)[1] > brown_bowl_pos[1]:
        use_block_names.append(block_name)
ret_val = use_block_names
'''
},
{
"role": "user",
"content": "objects = ['blue block', 'cyan block', 'purple bowl', 'brown bowl', 'purple block'] \n # the blue block"
},
{
"role": "assistant",
"content": '''
ret_val = 'blue block'
'''
},
{
"role": "user",
"content": "objects = ['blue block', 'cyan block', 'purple bowl', 'brown bowl', 'purple block'] \n # the block closest to the bottom right corner"
},
{
"role": "assistant",
"content": '''
corner_pos = parse_position('bottom right corner')
block_names = ['blue block', 'cyan block', 'purple block']
block_positions = get_object_center_positionitions_np(block_names)
closest_block_idx = np.argsort(np.linalg.norm(block_positions - corner_pos, axis=1))[0]
closest_block_name = block_names[closest_block_idx]
ret_val = closest_block_name
'''
},
{
"role": "user",
"content": "objects = ['brown bowl', 'green block', 'brown block', 'green bowl', 'blue bowl', 'blue block'] \n # the left most block"
},
{
"role": "assistant",
"content": '''
block_names = ['green block', 'brown block', 'blue block']
block_positions = get_object_center_positionitions_np(block_names)
left_block_idx = np.argsort(block_positions[:, 0])[0]
left_block_name = block_names[left_block_idx]
ret_val = left_block_name
'''
},
{
"role": "user",
"content": "objects = ['brown bowl', 'green block', 'brown block', 'green bowl', 'blue bowl', 'blue block'] \n # the bowl on near the top"
},
{
"role": "assistant",
"content": '''
bowl_names = ['brown bowl', 'green bowl', 'blue bowl']
bowl_positions = get_object_center_positionitions_np(bowl_names)
top_bowl_idx = np.argsort(block_positions[:, 1])[-1]
top_bowl_name = bowl_names[top_bowl_idx]
ret_val = top_bowl_name
'''
},
{
"role": "user",
"content": "objects = ['yellow bowl', 'purple block', 'yellow block', 'purple bowl', 'pink bowl', 'pink block'] \n # the third bowl from the right"
},
{
"role": "assistant",
"content": '''
bowl_names = ['yellow bowl', 'purple bowl', 'pink bowl']
bowl_positions = get_object_center_positionitions_np(bowl_names)
bowl_idx = np.argsort(block_positions[:, 0])[-3]
bowl_name = bowl_names[bowl_idx]
ret_val = bowl_name
'''
},
{# parse the drawer instance
"role": "user",
"content": "objects = ['cabinet', 'drawer_0', 'drawer_1', 'drawer_2', 'drawer_3', 'table'] \n # the drawer closest to the table"
},
{
"role": "assistant",
"content": '''
drawer_names = ['drawer_0', 'drawer_1', 'drawer_2', 'drawer_3']
table_pos = get_object_center_position('table')
drawer_positions = get_object_center_positionitions_np(drawer_names)
closest_drawer_idx = np.argsort(np.linalg.norm(drawer_positions - table_pos, axis=1))[0]
closest_drawer_name = drawer_names[closest_drawer_idx]
ret_val = closest_drawer_name
'''
},
{
"role": "user",
"content": "objects = ['cabinet', 'drawer_0', 'drawer_1', 'drawer_2', 'drawer_3', 'table'] \n # the top-most drawer"
},
{
"role": "assistant",
"content": '''
drawer_names = ['drawer_0', 'drawer_1', 'drawer_2', 'drawer_3']
drawer_positions = get_object_center_positionitions_np(drawer_names)
top_drawer_idx = np.argsort(drawer_positions[:, 2])[-1]
top_drawer_name = drawer_names[top_drawer_idx]
ret_val = top_drawer_name
'''
},
{
"role": "user",
"content": "objects = ['cabinet', 'drawer_0', 'drawer_1', 'drawer_2', 'drawer_3', 'table'] \n # the second drawer counting from the top"
},
{
"role": "assistant",
"content": '''
drawer_names = ['drawer_0', 'drawer_1', 'drawer_2', 'drawer_3']
drawer_positions = get_object_center_positionitions_np(drawer_names)
second_drawer_idx = np.argsort(drawer_positions[:, 2])[-2]
second_drawer_name = drawer_names[second_drawer_idx]
ret_val = second_drawer_name
'''
}
]

message_transform_shape_pts = [
{
"role":"system",
"content": '''
You are a strict assistant, translating natural language to a python script to control a robotic arm.
Your output should be a python script that can be executed to transform the shape of points indicated by instructions.
'''
},
{
"role": "assistant",
"content": '''
import numpy as np
from utils import get_object_center_position, get_obj_name_list, parse_pose, get_object_center_position
'''
},
{
"role": "user",
"content": "# move it to the right by 10cm"
},
{
"role": "assistant",
"content": "new_shape_pts = translate_pts_np(shape_pts, delta=[0.1, 0])"
},
{
"role": "user",
"content": "# move it to the top by 20cm"
},
{
"role": "assistant",
"content": "new_shape_pts = translate_pts_np(shape_pts, delta=[0, 0.2])"
},
{
"role": "user",
"content": "# rotate it clockwise by 40 degrees"
},
{
"role": "assistant",
"content": "new_shape_pts = rotate_pts_around_centroid_np(shape_pts, angle=-np.deg2rad(40))"
},
{
"role": "user",
"content": "# rotate by 30 degrees and make it slightly smaller"
},
{
"role": "assistant",
"content": '''
new_shape_pts = rotate_pts_around_centroid_np(shape_pts, angle=np.deg2rad(30))
new_shape_pts = scale_pts_around_centroid_np(new_shape_pts, scale_x=0.7, scale_y=0.7)
'''
},
{
"role": "user",
"content": "# move it toward the blue block"
},
{
"role": "assistant",
"content": '''
block_name = parse_obj_name('the blue block', f'objects = {get_obj_name_list()}')
block_pos = get_object_center_position(block_name)
mean_delta = np.mean(block_pos - shape_pts, axis=1)
new_shape_pts = translate_pts_np(shape_pts, mean_delta)
'''
}
]

message_fgen= [
{
"role":"system",
"content": '''
You are a strict assistant, translating natural language to a python script to define python functions.
The chat history provides some examples of how to define functions.
Your generated content should only contain a single Python function and comments starting with '#'. No questions or common texts.
'''
},
{
"role":"user",
"content": "# define function: open_gripper()"
},
{# def pick_and_place(pick_pose, place_pose)
"role":"user",
"content": "# define function: pick_and_place(object_name, pick_pose, place_pose)"
},
{
"role":"assistant",
"content": 
'''
def pick_and_place(pick_pose, place_pose):
    move_to_pose(pick_pose)
    close_gripper()
    attach_object(object_name)
    move_to_pose(place_pose)
    open_gripper()
    detach_object(object_name)
'''
},
{
"role":"user",
"content": "# define function: generate_arc_path(current_pose, center, radius, n, angle)"
},
{
"role":"assistant",
"content": 
'''
def generate_arc_path(current_pose, center, radius, n, angle):
    waypoints = []
    for i in range(n):
        waypoint = Pose()
        waypoint.position.x = center[0] + radius * np.cos(angle / n * i)
        waypoint.position.y = center[1] + radius * np.sin(angle / n * i)
        waypoint.position.z = center[2]
        waypoint.orientation = current_pose.orientation
        waypoints.append(waypoint)
    return waypoints
'''
},
{
"role":"user",
"content": "# define function: follow_path(move_group, path_points)"
},
{
"role":"assistant",
"content":
'''
def follow_path(move_group, path_points):
    (plan, fraction) = move_group.compute_cartesian_path(path_points, 0.01, 0.0)
    move_group.execute(plan, wait=True)
'''
},
{
"role":"user",
"content": "# define function: move_in_direction(axis, distance)"
},
{
"role": "assistant",
"content":
'''
def move_in_direction(axis: np.array, distance: float):
    current_pose = get_gripper_pose()
    target_pose = Pose()
    normalized_axis = np.array(axis) / np.linalg.norm(axis)
    target_pose.position.x = axis[0] * distance + current_pose.position.x
    target_pose.position.y = axis[1] * distance + current_pose.position.y
    target_pose.position.z = axis[2] * distance + current_pose.position.z
    target_pose.orientation = current_pose.orientation
    move_to_pose(target_pose)
'''
}
]

