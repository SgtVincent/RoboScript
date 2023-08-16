# %%
"""Python interface to execute grasps on the Franka Emika panda robot.

This module provides a class to execute grasps on the Franka Emika panda robot.
It uses MoveIt! to plan and execute the grasps.
"""
from typing import List, NamedTuple
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from visualization_msgs.msg import Marker


import moveit_commander
from visualization_msgs.msg import Marker

import trimesh

import actionlib
import numpy as np
from scipy.spatial.transform import Rotation
import franka_gripper.msg
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
import rospy

# Brings in the SimpleActionClient
import actionlib
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from src.env.moveit_interface import GraspExecutor

# %%

rospy.init_node("grasp_executor")
grasp_controller = GraspExecutor("world", load_wall=False, reset_pose = [-0.39999520574177366, 0.4740489757251173, -0.5709029256494976, -1.1338590764153493, 0.7967397934201639, 1.0726936823311086, -1.427030496828192])
print("Resetted")
grasp_controller.reset() 
pos = [0.15 + 0.3, 0.10, 0.6]
ori = np.array([0, -0.923, 0.382, 0])
ori = ori / np.linalg.norm(ori)
print("Moving to", pos)
# grasp_controller.move_to_pose(pos, ori)
# grasp_controller.grasp(rospy.get_param("~grasp_position"), rospy.get_param("~grasp_orientation"), dryrun = rospy.get_param("~sim"), object_id = None)
#grasp_controller.reset()


# for _ in range(3):
#     sphere = trimesh.primitives.Sphere(radius=0.05)
#     sphere.apply_translation([0.3,0.3,0.2+_*0.05])
#     grasp_controller.register_object(sphere, _)

# while not rospy.is_shutdown():
#     grasp_controller.reset()
#     grasp_controller.grasp([0.3,0.3,0.4], [1,0,0,0], dryrun = True, object_id = 2)
#     grasp_controller.reset()

# print("done")
rospy.spin()
