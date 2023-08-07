#### LMP Prompts ######

messages_system_default = [{
"role":"system",
"content": '''
You are a strict assistant, translating natural language to a python script to control a robotic arm.
Your output should be a python script that can be executed to perform the task described in the input.
'''
}]

message_tabletop_ui = [
{
"role":"assistant",
"content": '''
import numpy as np
from env_utils import put_first_on_second, get_obj_pos, get_obj_names, say, get_corner_name, get_side_name, is_obj_visible, stack_objects_in_order
from plan_utils import parse_obj_name, parse_position, parse_question, transform_shape_pts  
'''
}, 
{
"role":"user",
"content": "objects = ['yellow block', 'green block', 'yellow bowl', 'blue block', 'blue bowl', 'green bowl'] \n # put the yellow block on the yellow bowl"
},
{
"role":"assistant",
"content": '''
say('Ok - putting the yellow block on the yellow bowl')
put_first_on_second('yellow block', 'yellow bowl')
'''
},
{
"role":"user",
"content": "objects = ['yellow block', 'green block', 'yellow bowl', 'blue block', 'blue bowl', 'green bowl'] \n # which block did you move",
},
{
"role":"assistant",
"content": '''
say('I moved the yellow block')
'''
},
{
"role":"user",
"content": "objects = ['yellow block', 'green block', 'yellow bowl', 'blue block', 'blue bowl', 'green bowl'] \n # move the green block to the top right corner",
},
{
"role":"assistant",
"content": '''
say('Got it - putting the green block on the top right corner')
corner_pos = parse_position('top right corner')
put_first_on_second('green block', corner_pos)
'''
},
{
"role":"user",
"content": "objects = ['yellow block', 'green block', 'yellow bowl', 'blue block', 'blue bowl', 'green bowl'] \n # stack the blue bowl on the yellow bowl on the green block",
},
{
"role":"assistant",
"content": '''
order_bottom_to_top = ['green block', 'yellow block', 'blue bowl']
say(f'Sure - stacking from top to bottom: {", ".join(order_bottom_to_top)}')
stack_objects_in_order(object_names=order_bottom_to_top)
'''
},
{
"role":"user",
"content": "objects = ['cyan block', 'white block', 'cyan bowl', 'blue block', 'blue bowl', 'white bowl'] \n # move the cyan block into its corresponding bowl",
},
{
"role":"assistant",
"content": '''
matches = {'cyan block': 'cyan bowl'}
say('Got it - placing the cyan block on the cyan bowl')
for first, second in matches.items():
  put_first_on_second(first, get_obj_pos(second))
'''
},
{
"role":"user",
"content": "objects = ['cyan block', 'white block', 'cyan bowl', 'blue block', 'blue bowl', 'white bowl'] \n # make a line of blocks on the right side",
},
{
"role":"assistant",
"content": '''
say('No problem! Making a line of blocks on the right side')
block_names = parse_obj_name('the blocks', f'objects = {get_obj_names()}')
line_pts = parse_position(f'a 30cm vertical line on the right with {len(block_names)} points')
for block_name, pt in zip(block_names, line_pts):
  put_first_on_second(block_name, pt)
'''
},
{
"role":"user",
"content": "objects = ['yellow block', 'red block', 'yellow bowl', 'gray block', 'gray bowl', 'red bowl'] \n # put the small banana colored thing in between the blue bowl and green block",
},
{
"role":"assistant",
"content": '''
say('Sure thing - putting the yellow block between the blue bowl and the green block')
target_pos = parse_position('a point in the middle betweeen the blue bowl and the green block')
put_first_on_second('yellow block', target_pos)
'''
},
{
"role":"user",
"content": "objects = ['yellow block', 'red block', 'yellow bowl', 'gray block', 'gray bowl', 'red bowl'] \n # can you cut the bowls in half",
},
{
"role":"assistant",
"content": '''
say('no, I can only move objects around')
'''
},
{
"role":"user",
"content": "objects = ['yellow block', 'green block', 'yellow bowl', 'gray block', 'gray bowl', 'green bowl'] \n # stack the blocks on the right side with the gray one on the bottom"
},
{
"role":"assistant",
"content": '''
say('Ok. stacking the blocks on the right side with the gray block on the bottom')
right_side = parse_position('the right side')
put_first_on_second('gray block', right_side)
order_bottom_to_top = ['gray block', 'green block', 'yellow block']
stack_objects_in_order(object_names=order_bottom_to_top)
'''
},
{
"role":"user",
"content": "objects = ['yellow block', 'green block', 'yellow bowl', 'blue block', 'blue bowl', 'green bowl'] \n # hide the blue bowl"
},
{
"role":"assistant",
"content": '''
bowl_name = np.random.choice(['yellow bowl', 'green bowl'])
say(f'Sounds good! Hiding the blue bowl under the {bowl_name}')
put_first_on_second(bowl_name, 'blue bowl')
'''
},
{
"role":"user",
"content": "objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl'] \n # stack everything with the green block on top"
},
{
"role":"assistant",
"content": '''
say('Ok! Stacking everything with the green block on the top')
order_bottom_to_top = ['blue bowl', 'pink bowl', 'green bowl', 'pink block', 'blue block', 'green block']
stack_objects_in_order(object_names=order_bottom_to_top)
'''
},
{
"role":"user",
"content": "objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl'] \n # move the grass-colored bowl to the left"
},
{
"role":"assistant",
"content": '''
say('Sure - moving the green bowl left by 10 centimeters')
left_pos = parse_position('a point 10cm left of the green bowl')
put_first_on_second('green bowl', left_pos)
'''
},
{
"role":"user",
"content": "objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl'] \n # why did you move the red bowl"
},
{
"role":"assistant",
"content": '''
say(f'I did not move the red bowl')
'''
},
{
"role":"user",
"content": "objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl'] \n # undo that"
},
{
"role":"assistant",
"content": '''
say('Sure - moving the green bowl right by 10 centimeters')
left_pos = parse_position('a point 10cm right of the green bowl')
put_first_on_second('green bowl', left_pos)
'''
},
{
"role":"user",
"content": "objects = ['brown bowl', 'green block', 'brown block', 'green bowl', 'blue bowl', 'blue block'] \n # place the top most block to the corner closest to the bottom most block"
},
{
"role":"assistant",
"content": '''
top_block_name = parse_obj_name('top most block', f'objects = {get_obj_names()}')
bottom_block_name = parse_obj_name('bottom most block', f'objects = {get_obj_names()}')
closest_corner_pos = parse_position(f'the corner closest to the {bottom_block_name}', f'objects = {get_obj_names()}')
say(f'Putting the {top_block_name} on the {get_corner_name(closest_corner_pos)}')
put_first_on_second(top_block_name, closest_corner_pos)
'''
},
{
"role":"user",
"content": "objects = ['brown bowl', 'green block', 'brown block', 'green bowl', 'blue bowl', 'blue block'] \n # move the brown bowl to the side closest to the green block"
},
{
"role":"assistant",
"content": '''
closest_side_position = parse_position('the side closest to the green block')
say(f'Got it - putting the brown bowl on the {get_side_name(closest_side_position)}')
put_first_on_second('brown bowl', closest_side_position)
'''
},
{
"role":"user",
"content": "objects = ['brown bowl', 'green block', 'brown block', 'green bowl', 'blue bowl', 'blue block'] \n # move the brown bowl to the side closest to the green block"
},
{
"role":"assistant",
"content": '''
closest_side_position = parse_position('the side closest to the green block')
say(f'Got it - putting the brown bowl on the {get_side_name(closest_side_position)}')
put_first_on_second('brown bowl', closest_side_position)
'''
},
{
"role":"user",
"content": "objects = ['brown bowl', 'green block', 'brown block', 'green bowl', 'blue bowl', 'blue block'] \n # place the green block to the right of the bowl that has the blue block"
},
{
"role":"assistant",
"content": '''
bowl_name = parse_obj_name('the bowl that has the blue block', f'objects = {get_obj_names()}')
if bowl_name:
  target_pos = parse_position(f'a point 10cm to the right of the {bowl_name}')
  say(f'No problem - placing the green block to the right of the {bowl_name}')
  put_first_on_second('green block', target_pos)
else:
  say('There are no bowls that has the blue block')
'''
},
{
"role":"user",
"content": "objects = ['brown bowl', 'green block', 'brown block', 'green bowl', 'blue bowl', 'blue block'] \n # place the blue block in the empty bowl"
},
{
"role":"assistant",
"content": '''
empty_bowl_name = parse_obj_name('the empty bowl', f'objects = {get_obj_names()}')
if empty_bowl_name:
  say(f'Ok! Putting the blue block on the {empty_bowl_name}')
  put_first_on_second('blue block', empty_bowl_name)
else:
  say('There are no empty bowls')
'''
},
{
"role":"user",
"content": "objects = ['brown bowl', 'green block', 'brown block', 'green bowl', 'blue bowl', 'blue block'] \n # move the other blocks to the bottom corners"
},
{
"role":"assistant",
"content": '''
block_names = parse_obj_name('blocks other than the blue block', f'objects = {get_obj_names()}')
corners = parse_position('the bottom corners')
for block_name, pos in zip(block_names, corners):
  put_first_on_second(block_name, pos)
'''
},
{
"role":"user",
"content": "objects = ['brown bowl', 'green block', 'brown block', 'green bowl', 'blue bowl', 'blue block'] \n # move the red bowl a lot to the left of the blocks"
},
{
"role":"assistant",
"content": '''
say('Sure! Moving the red bowl to a point left of the blocks')
left_pos = parse_position('a point 20cm left of the blocks')
put_first_on_second('red bowl', left_pos)
'''
},
{
"role":"user",
"content": "objects = ['pink block', 'gray block', 'orange block'] \n # move the pinkish colored block on the bottom side"
},
{
"role":"assistant",
"content": '''
say('Ok - putting the pink block on the bottom side')
bottom_side_pos = parse_position('the bottom side')
put_first_on_second('pink block', bottom_side_pos)
'''
},
{
"role":"user",
"content": "objects = ['yellow bowl', 'blue block', 'yellow block', 'blue bowl'] \n # is the blue block to the right of the yellow bowl"
},
{
"role":"assistant",
"content": '''
if parse_question('is the blue block to the right of the yellow bowl?', f'objects = {get_obj_names()}'):
    say('yes, there is a blue block to the right of the yellow bow')
else:
    say('no, there is\'t a blue block to the right of the yellow bow')vv
'''
},
{
"role":"user",
"content": "objects = ['yellow bowl', 'blue block', 'yellow block', 'blue bowl'] \n # how many yellow objects are there"
},
{
"role":"assistant",
"content": '''
n_yellow_objs = parse_question('how many yellow objects are there', f'objects = {get_obj_names()}')
say(f'there are {n_yellow_objs} yellow object')
'''
},
{
"role":"user",
"content": "objects = ['yellow bowl', 'blue block', 'yellow block', 'blue bowl'] \n # move the left most block to the green bowl"
},
{    
"role":"assistant",
"content": '''
left_block_name = parse_obj_name('left most block', f'objects = {get_obj_names()}')
say(f'Moving the {left_block_name} on the green bowl') 
put_first_on_second(left_block_name, 'green bowl')
'''
},
{ 
"role":"user",
"content": "objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl'] \n # move the other blocks to different corners"
},
{
"role":"assistant",
"content": '''
block_names = parse_obj_name(f'blocks other than the {left_block_name}', f'objects = {get_obj_names()}')
corners = parse_position('the corners') 
say(f'Ok - moving the other {len(block_names)} blocks to different corners')
for block_name, pos in zip(block_names, corners):
  put_first_on_second(block_name, pos)  
'''
},
{
"role":"user",
"content": "objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl'] \n # is the pink block on the green bowl"
},
{
"role":"assistant",
"content": '''
if parse_question('is the pink block on the green bowl', f'objects = {get_obj_names()}'):
  say('Yes - the pink block is on the green bowl.')
else:
  say('No - the pink block is not on the green bowl.')  
'''
},
{ 
"role":"user",
"content": "objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl'] \n # what are the blocks left of the green bowl"
},
{
"role":"assistant",   
"content": '''
left_block_names =  parse_question('what are the blocks left of the green bowl', f'objects = {get_obj_names()}')
if len(left_block_names) > 0:
  say(f'These blocks are left of the green bowl: {", ".join(left_block_names)}')
else:
  say('There are no blocks left of the green bowl')
'''
},
{
"role":"user",
"content": "objects = ['pink block', 'green block', 'pink bowl', 'blue block', 'blue bowl', 'green bowl'] \n # if you see a purple bowl put it on the blue bowl"
},
{
"role":"assistant",
"content": '''
if is_obj_visible('purple bowl'):
  say('Putting the purple bowl on the pink bowl')
  put_first_on_second('purple bowl', 'pink bowl')
else:
  say('I don\'t see a purple bowl')
'''
},
{
"role":"user",
"content": "objects = ['pink block', 'gray block', 'orange block'] \n # imagine that the bowls are different biomes on earth and imagine that the blocks are parts of a building"
},
{
"role":"assistant",
"content": '''
say('ok')
'''
},
{
"role":"user",
"content": "objects = ['yellow block', 'green block', 'yellow bowl', 'blue block', 'blue bowl', 'green bowl'] \n # now build a tower in the grasslands"
},
{
"role":"assistant",
"content": '''
order_bottom_to_top = ['green bowl', 'blue block', 'green block', 'yellow block']
say('stacking the blocks on the green bowl')
stack_objects_in_order(object_names=order_bottom_to_top)
'''
},
{
"role":"user",
"content": "objects = ['yellow block', 'green block', 'yellow bowl', 'gray block', 'gray bowl', 'green bowl'] \n # show me what happens when the desert gets flooded by the ocean"
},
{ 
"role":"assistant",
"content": '''
say('putting the yellow bowl on the blue bowl')
put_first_on_second('yellow bowl', 'blue bowl')
'''
},
{
"role":"user",
"content": "objects = ['pink block', 'gray block', 'orange block'] \n # move all blocks 5cm toward the top"
},
{
"role":"assistant",
"content": '''
say('Ok - moving all blocks 5cm toward the top')
block_names = parse_obj_name('the blocks', f'objects = {get_obj_names()}')
for block_name in block_names:
  target_pos = parse_position(f'a point 5cm above the {block_name}')
  put_first_on_second(block_name, target_pos)
'''
},
{
"role":"user",
"content": "objects = ['cyan block', 'white block', 'purple bowl', 'blue block', 'blue bowl', 'white bowl'] \n # make a triangle of blocks in the middle"
},
{
"role":"assistant",
"content": '''
block_names = parse_obj_name('the blocks', f'objects = {get_obj_names()}')
triangle_pts = parse_position(f'a triangle with size 10cm around the middle with {len(block_names)} points')
say('Making a triangle of blocks around the middle of the workspace')
for block_name, pt in zip(block_names, triangle_pts):
  put_first_on_second(block_name, pt)
'''
},
{
"role":"user",
"content": "objects = ['cyan block', 'white block', 'purple bowl', 'blue block', 'blue bowl', 'white bowl'] \n # make the triangle smaller"
},
{
"role":"assistant",
"content": '''
triangle_pts = transform_shape_pts('scale it by 0.5x', shape_pts=triangle_pts)
say('Making the triangle smaller')
block_names = parse_obj_name('the blocks', f'objects = {get_obj_names()}')
for block_name, pt in zip(block_names, triangle_pts):
  put_first_on_second(block_name, pt)
'''
},
{
"role":"user",
"content": "objects = ['brown bowl', 'red block', 'brown block', 'red bowl', 'pink bowl', 'pink block'] \n # put the red block on the farthest bowl"
},
{
"role":"assistant",
"content": '''
farthest_bowl_name = parse_obj_name('the bowl farthest from the red block', f'objects = {get_obj_names()}')
say(f'Putting the red block on the {farthest_bowl_name}')
put_first_on_second('red block', farthest_bowl_name)
'''
}
]

message_parse_obj_name = [
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

message_fgen = [
{ 
"role":"assistant",
"content": '''
import numpy as np
from shapely.geometry import *
from shapely.affinity import *

from env_utils import get_obj_pos, get_obj_names
from ctrl_utils import put_first_on_second
'''
},
{
"role":"user",
"content": "# define function: total = get_total(xs=numbers)"
},
{
"role":"assistant",
"content": '''
def get_total(xs):
    return np.sum(xs)
'''
},
{
"role":"user",
"content": "# define function: y = eval_line(x, slope, y_intercept=0)"
},
{
"role":"assistant",
"content":'''
def eval_line(x, slope, y_intercept=0):
    return x * slope + y_intercept
'''
},
{
"role":"user",
"content": "# define function: pt = get_pt_to_the_left(pt, dist)"
},
{
"role":"assistant",
"content":'''
def get_pt_to_the_left(pt, dist):
    return pt + [-dist, 0]
'''
},
{
"role":"user",
"content": "# define function: pt = get_pt_to_the_top(pt, dist)"
},
{
"role":"assistant",
"content":'''
def get_pt_to_the_top(pt, dist):
    return pt + [0, dist]
'''
},
{
"role":"user",
"content": "# define function line = make_line_by_length(length=x)"
},
{
"role":"assistant",
"content":'''
def make_line_by_length(length):
    line = LineString([[0, 0], [length, 0]])
    return line 
'''
},
{
"role":"user",
"content": "# define function: line = make_vertical_line_by_length(length=x)"
},
{
"role":"assistant",
"content":'''
def make_vertical_line_by_length(length):
    line = make_line_by_length(length)
    vertical_line = rotate(line, 90)
    return vertical_line
'''
},
{
"role":"user",
"content": "# define function: pt = interpolate_line(line, t=0.5)"
},
{
"role":"assistant",
"content":'''
def interpolate_line(line, t):
    pt = line.interpolate(t, normalized=True)
    return np.array(pt.coords[0])
'''
},
{
"role":"user",
"content": "# example: scale a line by 2"
},
{
"role":"assistant",
"content":'''
line = make_line_by_length(1)
new_shape = scale(line, xfact=2, yfact=2)
'''
},
{
"role":"user",
"content": "# example: put object1 on top of object0"
},
{
"role":"assistant",
"content":'''
put_first_on_second('object1', 'object0')
'''
},
{
"role":"user",
"content": "# example: get the position of the first object"
},
{
"role":"assistant",
"content":'''
obj_names = get_obj_names()
pos_2d = get_obj_pos(obj_names[0])
'''
}
]
