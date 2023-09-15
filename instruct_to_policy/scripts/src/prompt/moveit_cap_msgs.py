'''
TODO: improve the message prompt in the following ways:
'''

message_tabletop_ui = [
{
"role":"system",
"content": '''
You are a strict assistant, translating natural language to a python script to control a robotic arm.
Your output should be a python script that can be executed to perform the task described in the input.
Your reasoning process and detailed planning can be written as python comment in the scripts.
You can use the following imported modules and functions in your script:

import moveit_commander
import actionlib
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from utils import get_obj_pos, get_obj_names, parse_obj_name, bbox_contains_pt, is_obj_visible, parse_pose, parse_question, transform_shape_pts, get_pose, get_bbox, get_color, get_corner_name, get_side_name, denormalize_xy
from plan_utils import add_object_to_scene, attach_object, detach_object, open_gripper, close_gripper, move_to_pose, move_joints_to, get_ee_pose,
'''
},
{# Open a Drawer
"role":"user",
"content": "objects = ['drawer', 'table', 'plants'] \n # open the drawer"
},
{
"role":"assistant",
"content":
'''
# Detailed planning: 
# Step 1: grasp the drawer 
# Step 2: pull the drawer 
# Step 3: release the drawer 

# Grasp the drawer
grasp_pose = parse_pose('drawer', action="grasp")
open_gripper(gripper_group)
move_to_pose(grasp_pose, move_group)
close_gripper(gripper_group)
attach_object('drawer')

# Pull the drawer
pull_pose = parse_pose('drawer', action="pull")  
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)

# Release the drawer
detach_object('drawer')
'''
},
{# Open a Door
"role":"user",
"content": "objects = ['door', 'handle'] \n # open the door"
},
{
"role":"assistant",
"content":
'''
# Detailed planning: 
# Step 1: grasp the door handle 
# Step 2: pull the door 
# Step 3: release the door handle 

# Grasp the door handle
grasp_pose = parse_pose('door', action="grasp")
open_gripper(gripper_group)
move_to_pose(grasp_pose, move_group)
close_gripper(gripper_group)
attach_object('door')

# Pull the door
pull_pose = parse_pose('door', action="pull")
move_to_pose(pull_pose, move_group)

# Release the door handle
open_gripper(gripper_group)
detach_object('door')
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
# Step 2: Turn the handle clockwise
# Step 3: Release the faucet handle

# Grasp the faucet handle
grasp_pose = parse_pose(object='faucet', action='grasp')
open_gripper(gripper_group)
move_to_pose(grasp_pose, move_group)
close_gripper(gripper_group)
attach_object('faucet')

# Turn the handle clockwise
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)

# Release the faucet handle
open_gripper(gripper_group)
detach_object('faucet')
'''
},
{# Press a Button
"role":"user",
"content": "objects = ['elevator button', 'elevator', 'plants'] \n # press the elevator button"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Close the gripper
# Step 2: Press the elevator button
# Step 3: Release the elevator button

# Close the gripper
close_gripper(gripper_group)

# Press the elevator button
attach_object('elevator button')
press_button_pose = parse_pose(object='elevator button', action='press')
move_to_pose(press_button_pose, move_group)

# Release the elevator button
release_button_pose = parse_pose(object='elevator button', action='release')
move_to_pose(release_button_pose, move_group)
open_gripper(gripper_group)
detach_object('elevator button')
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
# Step 1: Grasp the bottle
# Step 2: Pour the liquid into the glass
# Step 3: Place the bottle back on the table

# Get the position of the bottle
bottle_pos = get_obj_pos('bottle')

# Grasp the bottle
grasp_pose = parse_pose(object='bottle', action='grasp')
open_gripper(gripper_group)
move_to_pose(grasp_pose, move_group)
close_gripper(gripper_group)
attach_object('bottle')

# Pour the liquid into the glass
pour_pose = parse_pose(object='glass', action='pour')
move_to_pose(pour_pose, move_group)

# Place the bottle back on the table
table_pos = get_obj_pos('table')
place_pose = parse_pose(object='bottle', action='place', position=table_pos)
move_to_pose(place_pose, move_group)
open_gripper(gripper_group)
detach_object('bottle')
'''
},
{# Scanning an Object
"role":"user",
"content": "objects = ['bottle', 'glass', 'table'] \n # scan the bottle to get a 3D model"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Get the position of the bottle
# Step 2: Generate an orbit path around the bottle
# Step 3: Follow the orbit path to scan the bottle

# Get the position of the bottle
bottle_position = get_obj_pos('bottle')

# Generate an orbit path around the bottle
scan_path_points = generate_orbit_path(bottle_position, radius=0.1, n=20)

# Follow the orbit path to scan the bottle
follow_path(move_group, scan_path_points)
'''
},
{#Waving Hello
"role":"user",
"content": "objects = [] \n # wave hello for 3 times with the robot arm"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Define the start and end poses for the wave motion
# Step 2: Loop through the wave motion 3 times

# Define the start and end poses for the wave motion
wave_start_pose = parse_pose(object='', action='wave start')
wave_end_pose = parse_pose(object='', action='wave end')

# Loop through the wave motion 3 times
for _ in range(3):
    # Move to the start pose of the wave motion
    move_to_pose(wave_start_pose, move_group)    

    # Move to the end pose of the wave motion
    move_to_pose(wave_end_pose, move_group) 
'''
},
{# Folding clothing
"role":"user",
"content": "objects = ['clothing', 'table'] \n # fold the clothing"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Get the position of the clothing
# Step 2: Grasp the clothing with the robot arm
# Step 3: Move the clothing to the folding position
# Step 4: Release the clothing

# Get the position of the clothing
clothing_pos = get_obj_pos('clothing')

# Grasp the clothing with the robot arm
grab_clothing_pose = parse_pose(object='clothing', action='grab')
open_gripper(gripper_group)
move_to_pose(grab_clothing_pose, move_group)
close_gripper(gripper_group)
attach_object('clothing')

# Move the clothing to the folding position
fold_pose = parse_pose(object='clothing', action='fold')
move_to_pose(fold_pose, move_group)

# Release the clothing
open_gripper(gripper_group)
detach_object('clothing')
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
# Step 1: Define the start pose for the spoon
# Step 2: Grasp the spoon with the robot arm
# Step 3: Move to the bowl
# Step 4: Stir the ingredients in the bowl
# Step 5: Repeat step 4 for 5 times
# Step 6: Release the spoon

# Define the start pose for the spoon
grab_spoon_pose = parse_pose(object='spoon', action='grab')

# Grasp the spoon with the robot arm
open_gripper(gripper_group)
move_to_pose(grab_spoon_pose, move_group)
close_gripper(gripper_group)
attach_object('spoon')

# Move to the bowl
stir_pose = parse_pose(object='bowl', action='stir')
move_to_pose(stir_pose, move_group)

# Stir the ingredients in the bowl
# Repeat step 4 for 5 times
for _ in range(5):
    move_to_pose(grab_spoon_pose, move_group)
    move_to_pose(stir_pose, move_group)

# Release the spoon
open_gripper(gripper_group)
detach_object('spoon')
'''
},
{# Watering plants
"role":"user",
"content": "objects = ['watering can', 'plant'] \n # water the plant with the watering can"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Get the position of the watering can
# Step 2: Grasp the watering can with the robot arm
# Step 3: Move to the plant
# Step 4: Tilting the watering can to water the plant
# Step 5: Keep the watering can tilted for 5 seconds
# Step 6: Place back the watering can

# Get the position of the watering can
can_pos = get_obj_pos('watering can')

# Grasp the watering can with the robot arm
grasp_can_pose = parse_pose(object='watering can', action='grasp')
open_gripper(gripper_group)
move_to_pose(grasp_can_pose, move_group)
close_gripper(gripper_group)
attach_object('watering can')

# Move to the plant
watering_pose = parse_pose(object='plant', action='water')
move_to_pose(watering_pose, move_group)

# Tilting the watering can to water the plant
tilt_can_pose = parse_pose(object='watering can', action='tilt')
move_to_pose(tilt_can_pose, move_group)

# Keep the watering can tilted for 5 seconds
rospy.sleep(5)

# Place back the watering can
place_can_pose = parse_pose(object='watering can', action='place', position=can_pos)
move_to_pose(place_can_pose, move_group)
open_gripper(gripper_group)
detach_object('watering can')
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
# Step 1: Get the position of the bowl on the high shelf
# Step 2: Grasp the bowl with the robot arm
# Step 3: Move the bowl to the table
# Step 4: Put the bowl on the table

# Get the position of the bowl on the high shelf
bowl_pos = get_obj_pos('bowl', location='high shelf')

# Grasp the bowl with the robot arm
grasp_bowl_pose = parse_pose(object='bowl', action='grasp', description='a bowl from the high shelf')
open_gripper(gripper_group)
move_to_pose(grasp_bowl_pose, move_group)
close_gripper(gripper_group)
attach_object('bowl')

# Move the bowl to the table
put_bowl_pose = parse_pose(object='bowl', action='put', position='table')
move_to_pose(put_bowl_pose, move_group)

# Put the bowl on the table
open_gripper(gripper_group)
detach_object('bowl')
'''
},
{# Swap objects in the two containers
"role":"user",
"content": "objects = ['plate', 'fry_pan', 'table', 'peach', 'can'] \n # swap the peach and the can between the plate and the fry pan"
},
{
"role":"assistant",
"content":
'''
# Detailed planning:
# Step 1: Get the positions of the peach, can, plate, and fry pan
# Step 2: Grasp the peach with the robot arm
# Step 3: Move the peach to the fry pan
# Step 4: Release the peach
# Step 5: Grasp the can with the robot arm
# Step 6: Move the can to the plate
# Step 7: Release the can

# Get the positions of the peach, can, plate, and fry pan
peach_pos, can_pos, plate_pos, fry_pan_pos = get_obj_positions_np(['peach', 'can', 'plate', 'fry_pan'])

# Grasp the peach with the robot arm
peach_grasp_pose = parse_pose(object='peach', action='grasp')
open_gripper(gripper_group)
move_to_pose(peach_grasp_pose, move_group)
close_gripper(gripper_group)
attach_object('peach')

# Move the peach to the fry pan
peach_to_fry_pan_pose = parse_pose(object='peach', action='put', position=fry_pan_pos)
move_to_pose(peach_to_fry_pan_pose, move_group)

# Release the peach
open_gripper(gripper_group)
detach_object('peach')

# Grasp the can with the robot arm
can_grasp_pose = parse_pose(object='can', action='grasp')
move_to_pose(can_grasp_pose, move_group)
close_gripper(gripper_group)
attach_object('can')

# Move the can to the plate
can_to_plate_pose = parse_pose(object='can', action='put', position=plate_pos)
move_to_pose(can_to_plate_pose, move_group)

# Release the can
open_gripper(gripper_group)
detach_object('can')
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
from env_utils import get_obj_pos, parse_pose
from utils import get_obj_positions_np
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
block_positions = get_obj_positions_np(block_names)
closest_block_idx = np.argsort(np.linalg.norm(block_positions - get_obj_pos('purple bowl'), axis=1))[0]
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
brown_bowl_pos = get_obj_pos('brown bowl')
use_block_names = []
for block_name in block_names:
    if get_obj_pos(block_name)[1] > brown_bowl_pos[1]:
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
block_positions = get_obj_positions_np(block_names)
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
block_positions = get_obj_positions_np(block_names)
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
bowl_positions = get_obj_positions_np(bowl_names)
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
bowl_positions = get_obj_positions_np(bowl_names)
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
table_pos = get_obj_pos('table')
drawer_positions = get_obj_positions_np(drawer_names)
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
drawer_positions = get_obj_positions_np(drawer_names)
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
drawer_positions = get_obj_positions_np(drawer_names)
second_drawer_idx = np.argsort(drawer_positions[:, 2])[-2]
second_drawer_name = drawer_names[second_drawer_idx]
ret_val = second_drawer_name
'''
}
]

message_parse_question = [
{
"role":"system",
"content": '''
You are a strict assistant, translating natural language to a python script to return the answer to a question.
Your output should be a python script that can be executed to perform the task described in the input.
'''
},
{
"role": "assistant",
"content": '''
from utils import get_obj_pos, get_obj_names, parse_obj_name, bbox_contains_pt, is_obj_visible
'''
},
{
"role": "user",
"content": "objects = ['yellow bowl', 'blue block', 'yellow block', 'blue bowl', 'fruit', 'green block', 'black bowl'] \n # is the blue block to the right of the yellow bowl"
},
{
"role": "assistant",
"content": '''
ret_val = get_obj_pos('blue block')[0] > get_obj_pos('yellow bowl')[0]
'''
},
{
"role": "user",
"content": "objects = ['yellow bowl', 'blue block', 'yellow block', 'blue bowl', 'fruit', 'green block', 'black bowl'] \n # how many yellow objects are there"
},
{
"role": "assistant",
"content": '''
yellow_object_names = parse_obj_name('the yellow objects', f'objects = {get_obj_names()}')
ret_val = len(yellow_object_names)
'''
},
{
"role": "user",
"content": "objects = ['yellow bowl', 'blue block', 'yellow block', 'blue bowl', 'fruit', 'green block', 'black bowl'] \n # is the pink block on the green bowl"
},
{
"role": "assistant",
"content": '''
ret_val = bbox_contains_pt(container_name='green bowl', obj_name='pink block')
'''
},
{
"role": "user",
"content": "objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl'] \n # what are the blocks left of the green bowl"
},
{
"role": "assistant",
"content": '''
block_names = parse_obj_name('the blocks', f'objects = {get_obj_names()}')
green_bowl_pos = get_obj_pos('green bowl')
left_block_names = []
for block_name in block_names:
  if get_obj_pos(block_name)[0] < green_bowl_pos[0]:
    left_block_names.append(block_name) 
ret_val = left_block_names
'''
},
{
"role": "user",
"content": "objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl'] \n # is the sun colored block above the blue bowl?"
},
{
"role": "assistant",
"content": '''
sun_block_name = parse_obj_name('sun colored block', f'objects = {get_obj_names()}')
sun_block_pos = get_obj_pos(sun_block_name)
blue_bowl_pos = get_obj_pos('blue bowl')
ret_val = sun_block_pos[1] > blue_bowl_pos[1]
'''
},
{
"role": "user",
"content": "objects = ['pink block', 'yellow block', 'pink bowl', 'blue block', 'blue bowl', 'yellow bowl'] \n # is the green block below the blue bowl?"
},
{
"role": "assistant",
"content": '''
ret_val = get_obj_pos('green block')[1] < get_obj_pos('blue bowl')[1]
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
from utils import get_obj_pos, get_obj_names, parse_pose, get_obj_pos
'''
},
{
"role": "user",
"content": "# make it bigger by 1.5"
},
{
"role": "assistant",
"content": "new_shape_pts = scale_pts_around_centroid_np(shape_pts, scale_x=1.5, scale_y=1.5)" 
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
block_name = parse_obj_name('the blue block', f'objects = {get_obj_names()}')
block_pos = get_obj_pos(block_name)
mean_delta = np.mean(block_pos - shape_pts, axis=1)
new_shape_pts = translate_pts_np(shape_pts, mean_delta)
'''
}
]

# TODO: adapt to real API

message_fgen= [
{
"role":"system",
"content": '''
You are a strict assistant, translating natural language to a python script to define python functions.
Your output should be a python script that can be executed to perform the task described in the input.
'''
},
{
"role":"user",
"content": "# define function: open_gripper(gripper_group)"
},
# NOTE: open/close gripper are basic actions, so we don't need to define them ?
# {
# "role":"assistant",
# "content": '''
# def open_gripper(gripper_group):
#     gripper_group.set_named_target("open")
#     gripper_group.go(wait=True)
# '''
# },
# {
# "role":"user",
# "content": "# define function: close_gripper(gripper_group)"
# },
# {
# "role":"assistant",
# "content": '''
# def close_gripper(gripper_group):
#     gripper_group.set_named_target("closed")
#     gripper_group.go(wait=True)
# '''
# },
# TODO: add skills functions like grasp, place, pull, pick up, etc.
{ # Currently, parse_pose function is a naive implementation that only works for pick and place
# TODO: The pose grouding should be done by Large-scale multimodal model. 
"role":"user",
"content": "# define function: pose = parse_pose(object_name, action, description)"
},
{
"role":"assistant",
"content":
'''
def parse_pose(object, action, description):
    if action in ['pick', 'grasp', 'grab', 'get', 'take', 'hold']:
        pose = Pose()
        pose.position = get_obj_pos(object)
        pose.orientation = Quaternion(0,0,0,1)
    elif action in ['place', 'put', 'drop', 'release']:
        pose = Pose()
        pose.position = get_obj_pos(object)
        pose.position.z += 0.2
        pose.orientation = Quaternion(0,0,0,1)
    else:
        pose = Pose()
        pose.position = get_obj_pos(object)
        pose.orientation = Quaternion(0,0,0,1)
'''
},
{
"role":"user",
"content": "# define function: pick_and_place(move_group, pick_pose, place_pose)"
},
{
"role":"assistant",
"content": 
'''
def pick_and_place(move_group, pick_pose, place_pose):
    waypoints = [move_group.get_current_pose().pose, pick_pose, place_pose]
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    move_group.execute(plan, wait=True)
'''
},
{
"role":"user",
"content": "# define function: follow_path(move_group, pose)"
},
{
"role":"assistant",
"content":
'''
def follow_path(move_group, path_points):
    waypoints = [set_pose(1.0, point[0], point[1], point[2]) for point in path_points]
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    move_group.execute(plan, wait=True)
'''
}
]

