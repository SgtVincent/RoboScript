#!/usr/bin/env python3
import os 
import numpy as np
import openai
import cv2

# from google.colab.patches import cv2_imshow
from moviepy.editor import ImageSequenceClip
# from src.prompt.message_definitions import *
from src.lmp import *
# from src.env.pybullet_env import PickPlaceEnv
from src.env.moveit_gazebo_env import MoveitGazeboEnv
from src.configs.config import load_config
cfg_tabletop = load_config("perception_few_shot_gpt_3.5.yaml")
from src.openai_api_key import OPENAI_API_KEY
import rospy 
openai.api_key = OPENAI_API_KEY


# %% [markdown]
# # Interactive Tabletop Manipulation
# 
# Instructions: 
# 
# 1. Set the number of blocks and bowls with the sliders in the next cell.
# 2. Input a command in the `user_input` field below and run the cell. This will run Code as Policies by querying the code-generating LLM to write robot code to complete the task.
# 
# Note the object names printed after the Initilize Env cell - these are the objects in the scene and can be referred to in the commands.
# 
# Supported commands:
# * Spatial reasoning (e.g. to the left of the red block, the closest corner, the farthest bowl, the second block from the right)
# * Sequential actions (e.g. put blocks in matching bowls, stack blocks on the bottom right corner)
# * Contextual commands (e.g. do the same with the blue block, undo that)
# * Language-based reasoning (e.g. put the forest-colored block on the ocean-colored bowl).
# * Simple Q&A (e.g. how many blocks are to the left of the blue bowl?)
# 
# Example commands (note object names may need to be changed depending the sampled object names):
# * put the sun-colored block on the bowl closest to it
# * stack the blocks on the bottom most bowl
# * arrange the blocks as a square in the middle
# * move the square 5cm to the right
# * how many blocks are to the right of the orange bowl?
# * pick up the block closest to the top left corner and place it on the bottom right corner
# 
# Known limitations:
# * In simulation we're using ground truth object poses instead of using vision models. This means that commands the require knowledge of visual apperances (e.g. darkest bowl, largest object) are not supported.
# * Currently, the low-level pick place primitive does not do collision checking, so if there are many objects on the table, placing actions may incur collisions.
# * Prompt saturation - if too many commands (10+) are executed in a row, then the LLM may start to ignore examples in the early parts of the prompt.
# * Ambiguous instructions - if a given instruction doesn't lead to the desired actions, try rephrasing it to remove ambiguities (e.g. place the block on the closest bowl -> place the block on its closest bowl)

# %%
#@title Initialize Env { vertical-output: true }
num_blocks = 3 #@param {type:"slider", min:0, max:4, step:1}
num_bowls = 3 #@param {type:"slider", min:0, max:4, step:1}
high_resolution = False #@param {type:"boolean"}
high_frame_rate = False #@param {type:"boolean"}

# setup env and LMP
rospy.init_node('demo_moveit_cap', anonymous=True, log_level=rospy.WARN)
env = MoveitGazeboEnv(cfg_tabletop)
env.reset()

lmp_tabletop_ui:LMP = setup_LMP(env, cfg_tabletop)

# display env
# cv2_imshow(cv2.cvtColor(env.get_camera_image(), cv2.COLOR_BGR2RGB)) # for Google Colab
# import matplotlib.pyplot as plt # for local
# plt.imshow(cv2.cvtColor(env.get_camera_image(), cv2.COLOR_BGR2RGB))
print('available objects:')
print(env.object_names)

# %%
#@title Interactive Demo { vertical-output: true }

# user_input = 'put the blue block on the yellow bowl' #@param {allow-input: true, type:"string"}

# env.cache_video = []

# print('Running policy and recording video...')
# lmp_tabletop_ui(user_input, f'objects = {env.object_list}')

# user_input = "move the can from plate to table, and move the peach from table to plate"
user_input = "grasp the can from plate to table, and grasp the peach from table to plate"
# user_input = "grasp and pull the top-most drawer"

# user_input = 'open the drawer and check the number of bottles' #@param {allow-input: true, type:"string"}
# object_list = ['drawer', 'table']
# user_input = 'put all the objects on the table into the box'
# object_list = ['box', 'table', 'orange', 'apple', 'stone']

env.cache_video = []
print('Running policy and recording video...')
object_list = env.object_names
lmp_tabletop_ui(user_input, f'objects = {object_list}')

