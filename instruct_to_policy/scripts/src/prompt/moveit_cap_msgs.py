'''
TODO: improve the message prompt in the following ways:
'''

message_tabletop_ui = [
{
"role":"system",
"content": '''
You are a strict assistant, translating natural language to a python script to control a robotic arm.
Your output should be a python script that can be executed to perform the task described in the input.
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
"content": "objects = ['drawer', 'handle', 'table'] \n # open the drawer"
},
{
"role":"assistant",
"content":
'''
grasp_pose = parse_pose('drawer', action="grasp")
open_gripper(gripper_group)
move_to_pose(grab_handle_pose, move_group)
close_gripper(gripper_group)
attach_object('drawer')
pull_pose = parse_pose('drawer', action="pull")
move_to_pose(pull_pose, move_group)
open_gripper(gripper_group)
detach_object('drawer')
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
grasp_pose = parse_pose(object='faucet', action='grasp')
open_gripper(gripper_group)
move_to_pose(grasp_pose, move_group)
close_gripper(gripper_group)
attach_object('faucet')
waypoints = [move_group.get_current_pose().pose]
for angle in range(0, 91, 10):
    target_pose = set_pose(math.cos(math.radians(angle/2)), 0, 0, math.sin(math.radians(angle/2)))
    target_pose.position = current_pose.position
    waypoints.append(target_pose)
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)
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
press_button_pose = parse_pose(object='elevator button', action='press')
close_gripper(gripper_group)
move_to_pose(press_button_pose, move_group)

release_button_pose = parse_pose(object='elevator button', action='release')
move_to_pose(release_button_pose, move_group)
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
bottle_pos = get_obj_pos('bottle')

grasp_pose = parse_pose(object='bottle', action='grasp')
open_gripper(gripper_group)
move_to_pose(grasp_pose, move_group)
close_gripper(gripper_group)
attach_object('bottle')

pour_pose = parse_pose(object='glass', action='pour')
move_to_pose(pour_pose, move_group)

release_pose = parse_pose(object='bottle', action='release', position=bottle_pos)
move_to_pose(release_pose, move_group)
open_gripper(gripper_group)
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
bottle_position = get_obj_pos('bottle')
scan_path_points = generate_orbit_path(bottle_position, radius=0.1, n=10)
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
wave_start_pose = parse_pose(object='', action='wave start')
wave_end_pose = parse_pose(object='', action='wave end')
for _ in range(3):
    move_to_pose(wave_start_pose, move_group)
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
grab_clothing_pose = parse_pose(object='clothing', action='grab')
open_gripper(gripper_group)
move_to_pose(grab_clothing_pose, move_group)
close_gripper(gripper_group)
attach_object('clothing')
fold_pose = parse_pose(object='clothing', action='fold')
move_to_pose(fold_pose, move_group)
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
grab_spoon_pose = parse_pose(object='spoon', action='grab')
open_gripper(gripper_group)
move_to_pose(grab_spoon_pose, move_group)
close_gripper(gripper_group)
attach_object('spoon')

for _ in range(5):
    stir_pose = parse_pose(object='bowl', action='stir')
    move_to_pose(stir_pose, move_group)
    move_to_pose(grab_spoon_pose, move_group)

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
can_pos = get_obj_pos('watering can')
grasp_can_pose = parse_pose(object='watering can', action='grasp')
open_gripper(gripper_group)
move_to_pose(grab_can_pose, move_group)
close_gripper(gripper_group)
attach_object('watering can')

watering_pose = parse_pose(object='plant', action='water')
move_to_pose(watering_pose, move_group)

release_can_pose = parse_pose(object='watering can', action='release', position=can_pos)
move_to_pose(release_can_pose, move_group)
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
grasp_bowl_pose = parse_pose(object='bowl', action='grasp', description='a bowl from the high shelf')
open_gripper(gripper_group)
move_to_pose(bowl_pose, move_group)
close_gripper(gripper_group)
attach_object('bowl')

put_bowl_pose = parse_pose(object='bowl', action='put', description='a bowl on the table')
move_to_pose(put_bowl_pose, move_group)
open_gripper(gripper_group)
detach_object('bowl')
'''
}
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

