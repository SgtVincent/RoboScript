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
from utils import get_object_center_position, get_obj_name_list, parse_obj_name, bbox_contains_pt, 
'''
},
{
"role": "user",
"content": "objects = ['yellow bowl', 'blue block', 'yellow block', 'blue bowl', 'fruit', 'green block', 'black bowl'] \n # is the blue block to the right of the yellow bowl"
},
{
"role": "assistant",
"content": '''
ret_val = get_object_center_position('blue block')[0] > get_object_center_position('yellow bowl')[0]
'''
},
{
"role": "user",
"content": "objects = ['yellow bowl', 'blue block', 'yellow block', 'blue bowl', 'fruit', 'green block', 'black bowl'] \n # how many yellow objects are there"
},
{
"role": "assistant",
"content": '''
yellow_object_names = parse_obj_name('the yellow objects', f'objects = {get_obj_name_list()}')
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
block_names = parse_obj_name('the blocks', f'objects = {get_obj_name_list()}')
green_bowl_pos = get_object_center_position('green bowl')
left_block_names = []
for block_name in block_names:
  if get_object_center_position(block_name)[0] < green_bowl_pos[0]:
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
sun_block_name = parse_obj_name('sun colored block', f'objects = {get_obj_name_list()}')
sun_block_pos = get_object_center_position(sun_block_name)
blue_bowl_pos = get_object_center_position('blue bowl')
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
ret_val = get_object_center_position('green block')[1] < get_object_center_position('blue bowl')[1]
'''
}
]
