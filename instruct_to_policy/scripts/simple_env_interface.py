# %%
import os 
import sys
import numpy as np
import argparse
from scipy.spatial.transform import Rotation as R 
import open3d as o3d 
# add catkin_ws context 
sys.path.append("/home/junting/catkin_ws/devel/lib/python3.9/site-packages")
sys.path.append("/home/junting/franka_ws/devel/lib/python3.9/site-packages")

from src.lmp import *
from src.env.true_grounding_env import TrueGroundingEnv
from src.configs.config import load_config
cfg_tabletop = load_config("text_gpt_3.yaml")
import rospy 
import rospkg
import jupyros as jr

from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


# %%
########################################
# initialize environment
########################################
rospy.init_node('eval_code', log_level=rospy.DEBUG)
# get package root path 
pkg_root = rospkg.RosPack().get_path('instruct_to_policy')

# setup environment
env = TrueGroundingEnv(cfg_tabletop)
# env.reset()


# %%
rospkg.RosPack().get_path("franka_umi_description")



# %%
env.reset()

# %%
env.open_gripper()


# %%
pose_msg = env.parse_central_lift_grasp_pose("orange")

# %%
env.grasp(pose_msg)


# %%
env.close_gripper()

# %%
env.attach_object("orange")

place_pose = env.parse_place_pose(object_name="orange", receptacle_name="plate")

env.move_to_pose(place_pose)

env.open_gripper()

rospy.spin()