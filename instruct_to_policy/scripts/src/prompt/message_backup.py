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

