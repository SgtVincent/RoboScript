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
import franka_fisher.instruct_to_policy.scripts.src.env.moveit_env as moveit_env
importlib.reload(moveit_env)
import src.env.utils
importlib.reload(src.env.utils)
from src.env.utils import pose_to_list

# %%
import moveit_commander
moveit_commander.MoveGroupCommander("panda_manipulator").get_current_joint_values()

# %%


# %%
init_pose = [
-0.0001513652608586824,
-0.7856258935003497,
9.991577917034533e-06,
-2.3559505836972523,
2.0776787743592706e-05,
1.5717418653690114,
0.7853832208311591
]
grasp_controller = moveit_env.GraspExecutor("world", reset_pose=init_pose)
print("Resetted")
grasp_controller.reset() 


# %%
grasp_controller.objects


# %%
gazebo_node_name = 'gazebo'
object_name = 'stone'
frame_id = 'world'
get_model_state = rospy.ServiceProxy(f'{gazebo_node_name}/get_model_state', GetModelState)


# %%
object_state = get_model_state(object_name, frame_id)
object_pose = object_state.pose
print(object_pose)
pos, rot = pose_to_list(object_pose)
# print(pos, rot)

# %%
grasp_controller.group.get_current_pose()

# %%
grasp_controller.grasp(np.array(pos) + np.array([0,0,0.01], dtype=float), np.array([1, 0, 0, 0], dtype=float))
grasp_controller.move_to_pose(np.array(pos) + np.array([0,0,0.1], dtype=float), np.array([1,0,0,0], dtype=float))

# %%
rospy.spin()