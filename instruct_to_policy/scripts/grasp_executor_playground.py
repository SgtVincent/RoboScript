#!/usr/bin/env python3
# %%
from typing import List, NamedTuple
import numpy as np
import os 
from visualization_msgs.msg import Marker, MarkerArray
from visualization_msgs.msg import Marker
import moveit_commander
from visualization_msgs.msg import Marker
import numpy as np
from scipy.spatial.transform import Rotation
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
import rospy
import rospkg
from gazebo_msgs.srv import (
    GetModelState,
    GetWorldProperties,
    GetModelProperties,
)
from geometry_msgs.msg import PoseStamped


# %%
rospy.init_node("grasp_executor")


# %%
import importlib
import src.env.moveit_env as moveit_env
importlib.reload(moveit_env)
import src.env.utils
importlib.reload(src.env.utils)
from src.env.utils import pose_to_list
from src.config import cfg_tabletop
# %%
# import moveit_commander
# moveit_commander.MoveGroupCommander("panda_manipulator").get_current_joint_values()

# %%


# %%


env = moveit_env.MoveitGazeboEnv(cfg_tabletop)
print("Resetted")
env.reset() 


# %%
env.objects


# %%
gazebo_node_name = 'gazebo'
object_name = 'stone'
frame_id = 'world'
get_model_state = rospy.ServiceProxy(f'{gazebo_node_name}/get_model_state', GetModelState)


# %%
grasp_pose = env.parse_pose(object="stone", action="grasp")
env.open_gripper()
env.move_to_pose(grasp_pose)
env.close_gripper()


# %%
rospy.spin()