'''
TODO: improve the message prompt in the following ways:
'''

message_tabletop_ui = [
{
"role":"system",
"content": '''
You are a strict assistant, translating natural language to a python script to control a robotic arm.
Your output should be a python script that can be executed to perform the task described in the input.
'''
},
{
"role":"assistant",
"content":
'''
import moveit_commander
import actionlib
import rospy

from franka_gripper.msg import GraspAction, MoveAction
import geometry_msgs, std_msgs

from utils import get_obj_pos, get_obj_names, parse_obj_name, bbox_contains_pt, is_obj_visible
from plan_utils import parse_obj_name, parse_position, parse_pose, parse_question, transform_shape_pts  

rospy.init_node('pandas_moveit', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
manipulator = moveit_commander.MoveGroupCommander("panda_manipulator", wait_for_servers = 15)
gripper = moveit_commander.MoveGroupCommander("panda_hand", wait_for_servers = 15)
grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
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
grab_handle_pose = get_pose('handle')
open_gripper(gripper_command_group)
move_group.set_pose_target(grab_handle_pose)
move_group.go(wait=True)
pull_drawer_pose = get_pose('d)
move_group.set_pose_target(pull_drawer_pose)
move_group.go(wait=True)
open_gripper(gripper_command_group)
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
grab_faucet_pose = parse_pose(object='faucet', action='grab')
open_gripper(gripper_command_group)
move_group.set_pose_target(grab_faucet_pose)
move_group.go(wait=True)
close_gripper(gripper_command_group)
waypoints = [move_group.get_current_pose().pose]
for angle in range(0, 91, 10):
    target_pose = set_pose(math.cos(math.radians(angle/2)), 0, 0, math.sin(math.radians(angle/2)))
    target_pose.position = current_pose.position
    waypoints.append(target_pose)
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
move_group.execute(plan, wait=True)
open_gripper(gripper_command_group)
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
move_group.set_pose_target(button_pose)
move_group.go(wait=True)
close_gripper(gripper_command_group)
release_button_pose = parse_pose(object='elevator button', action='release')
move_group.set_pose_target(button_pose)
move_group.go(wait=True)
open_gripper(gripper_command_group)
'''
},
{# Pouring Liquid
"role":"user",
"content": "objects = ['bottle', 'glass', 'table'] \n # grasp a bottle and pour the liquid into the glass"
},
{
"role":"assistant",
"content":
'''
grab_bottle_pose = parse_pose(object='bottle', action='grab')
open_gripper(gripper_command_group)
move_group.set_pose_target(grab_bottle_pose)
move_group.go(wait=True)
close_gripper(gripper_command_group)
pour_water_pose = parse_pose(object='glass', action='pour')
move_group.set_pose_target(pour_water_pose)
move_group.go(wait=True)
move_group.set_pose_target(grab_bottle_pose)
move_group.go(wait=True)
open_gripper(gripper_command_group)
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
    move_group.set_pose_target(wave_start_pose)
    move_group.go(wait=True)
    move_group.set_pose_target(wave_end_pose)
    move_group.go(wait=True)
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
open_gripper(gripper_command_group)
move_group.set_pose_target(grab_clothing_pose)
move_group.go(wait=True)
close_gripper(gripper_command_group)
fold_pose = parse_pose(object='clothing', action='fold')
move_group.set_pose_target(fold_pose)
move_group.go(wait=True)
open_gripper(gripper_command_group)
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
open_gripper(gripper_command_group)
move_group.set_pose_target(grab_spoon_pose)
move_group.go(wait=True)
close_gripper(gripper_command_group)

for _ in range(5):
    stir_pose = parse_pose(object='bowl', action='stir')
    move_group.set_pose_target(stir_pose)
    move_group.go(wait=True)
    move_group.set_pose_target(grab_spoon_pose)
    move_group.go(wait=True)

open_gripper(gripper_command_group)
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
grab_can_pose = parse_pose(object='watering can', action='grab')
watering_pose = parse_pose(object='plant', action='water')
open_gripper(gripper_command_group)
move_group.set_pose_target(grab_can_pose)
move_group.go(wait=True)
close_gripper(gripper_command_group)
move_group.set_pose_target(watering_pose)
move_group.go(wait=True)
move_group.set_pose_target(grab_can_pose)
move_group.go(wait=True)
open_gripper(gripper_command_group)
'''
},
{# Retrieve an item from a high shelf
"role":"user",
"content": "objects = ['shelf', 'bowl'] \n # retrieve a bowl from the high shelf"
},
{
"role":"assistant",
"content":
'''
shelf_pose = set_pose(1.0, 0.6, 0.4, 0.1)
open_gripper(gripper_command_group)
move_group.set_pose_target(shelf_pose)
move_group.go(wait=True)
close_gripper(gripper_command_group)
lower_pose = set_pose(1.0, 0.4, -0.2, 0.1)
move_group.set_pose_target(lower_pose)
move_group.go(wait=True)
open_gripper(gripper_command_group)
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
from env_utils import get_obj_pos, parse_position
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

message_parse_position = [
{
"role":"system",
"content": '''
You are a strict assistant, translating natural language to a python script to parse the positions indicated by instructions.
Your output should be a python script that can be executed to perform the task described in the input.
'''
},
{
"role": "assistant",
"content": '''
import numpy as np
from shapely.geometry import *
from shapely.affinity import *
from env_utils import denormalize_xy, parse_obj_name, get_obj_names, get_obj_pos
'''
},
{
"role": "user",
"content": "# a 30cm horizontal line in the middle with 3 points"
},
{
"role": "assistant",
"content": '''
middle_pos = denormalize_xy([0.5, 0.5])
start_pos = middle_pos + [-0.3/2, 0]
end_pos = middle_pos + [0.3/2, 0]
line = make_line(start=start_pos, end=end_pos)
points = interpolate_pts_on_line(line=line, n=3)
ret_val = points
'''
},
{
"role": "user",
"content": "# a 20cm vertical line near the right with 4 points"
},
{
"role": "assistant",
"content": '''    
middle_pos = denormalize_xy([1, 0.5]) 
start_pos = middle_pos + [0, -0.2/2]
end_pos = middle_pos + [0, 0.2/2]
line = make_line(start=start_pos, end=end_pos)
points = interpolate_pts_on_line(line=line, n=4)
ret_val = points
'''
},
{
"role": "user",
"content": "# a diagonal line from the top left to the bottom right corner with 5 points"
},
{
"role": "assistant",
"content": '''
top_left_corner = denormalize_xy([0, 1])
bottom_right_corner = denormalize_xy([1, 0])
line = make_line(start=top_left_corner, end=bottom_right_corner)
points = interpolate_pts_on_line(line=line, n=5)
ret_val = points
'''
},
{
"role": "user",
"content": "# a triangle with size 10cm with 3 points"
},
{
"role": "assistant",
"content": '''
polygon = make_triangle(size=0.1, center=denormalize_xy([0.5, 0.5]))
points = get_points_from_polygon(polygon)
ret_val = points
'''
},
{
"role": "user",
"content": "# the corner closest to the sun colored block"
},
{
"role": "assistant",
"content": '''
block_name = parse_obj_name('the sun colored block', f'objects = {get_obj_names()}')
corner_positions = np.array([denormalize_xy(pos) for pos in [[0, 0], [0, 1], [1, 1], [1, 0]]])
closest_corner_pos = get_closest_point(points=corner_positions, point=get_obj_pos(block_name))
ret_val = closest_corner_pos
'''
},
{
"role": "user",
"content": "# the side farthest from the right most bowl"
},
{
"role": "assistant",
"content": '''
bowl_name = parse_obj_name('the right most bowl', f'objects = {get_obj_names()}')
side_positions = np.array([denormalize_xy(pos) for pos in [[0.5, 0], [0.5, 1], [1, 0.5], [0, 0.5]]])
farthest_side_pos = get_farthest_point(points=side_positions, point=get_obj_pos(bowl_name))
ret_val = farthest_side_pos
'''
},
{
"role": "user",
"content": "# a point above the third block from the bottom"
},
{
"role": "assistant",
"content": '''
block_name = parse_obj_name('the third block from the bottom', f'objects = {get_obj_names()}')
ret_val = get_obj_pos(block_name) + [0.1, 0]
'''
},
{
"role": "user",
"content": "# a point 10cm left of the bowls"
},
{
"role": "assistant",
"content": '''
bowl_names = parse_obj_name('the bowls', f'objects = {get_obj_names()}')
bowl_positions = get_all_object_positions_np(obj_names=bowl_names)
left_obj_pos = bowl_positions[np.argmin(bowl_positions[:, 0])] + [-0.1, 0]
ret_val = left_obj_pos
'''
},
{
"role": "user",
"content": "# the bottom side"
},
{
"role": "assistant",
"content": '''
bottom_pos = denormalize_xy([0.5, 0])
ret_val = bottom_pos
'''
},
{
"role": "user",
"content": "# the top corners"
},
{
"role": "assistant",
"content": '''
top_left_pos = denormalize_xy([0, 1])
top_right_pos = denormalize_xy([1, 1])
ret_val = [top_left_pos, top_right_pos]
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
from utils import get_obj_pos, get_obj_names, parse_position, parse_obj_name
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
"content": "# define function: open_gripper(gripper_command_group)"
},
{
"role":"assistant",
"content": '''
def open_gripper(gripper_command_group):
    gripper_command_group.set_named_target("open")
    gripper_command_group.go(wait=True)
'''
},
{
"role":"user",
"content": "# define function: close_gripper(gripper_command_group)"
},
{
"role":"assistant",
"content": '''
def close_gripper(gripper_command_group):
    gripper_command_group.set_named_target("closed")
    gripper_command_group.go(wait=True)
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

