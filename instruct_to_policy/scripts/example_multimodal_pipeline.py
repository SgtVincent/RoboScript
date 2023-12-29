# %%
import os 
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R 
import rospy 
import rospkg
# import jupyros as jr

from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# %%
# add catkin_ws context 
# sys.path.append("/home/junting/franka_ws/devel/lib/python3.9/site-packages")

from src.lmp import *
from src.env.multimodal_env import MultiModalEnv
from src.config import cfg_tabletop


# [-0.7463056383716324, -0.9501271177342063, 0.05601069181248034, -2.689066292265651, -0.13831545807280166, 2.2432093219785463, 0.7685819521267404],
# [-1.9619020328228876, -0.7889417275044455, -0.12113141601441198, -3.0557122047782443, -0.030707268989748425, 3.293706556185751, 0.5389143775616523]
ITER_THRES = 2
pos_list = [            [0.7186019320366168, -1.2853656898536576, -0.732145829175648, -1.898301090742412, -0.2763346155094793, 1.7879803434213002, 0.15318722031172902]]

# %%
########################################
# initialize environment
########################################
rospy.init_node('jupyter_multimodal', log_level=rospy.DEBUG)
# get package root path 
pkg_root = rospkg.RosPack().get_path('instruct_to_policy')

# setup environment
env = MultiModalEnv(cfg_tabletop)


# %%
env.reset()
success = False
iter_time = 0

def example():
    # for i, reset_pos in enumerate(reset_pos_list):
    #     env.move_joints_to(reset_pos)
    #     if i == 0:
    #         image_dict = env.get_sensor_data()
    #     else:
    #         image_set = env.get_sensor_data()
    #         for key, value in image_dict.items():
    #             value.extend(image_set[key])
    #             image_dict[key] = value


    # %%
    print(os.getcwd())

    query = 'open the drawer'
    object_list = ["handle"]
    data = {
            'pos_list': pos_list
        }
    
    env.scene.update_pos_list(data)

    # %%
    env.detect_objects_with_moving(pos_list = pos_list, object_list=["handle"])

    # %%
    env.scene.visualize_3d_bboxes()
    obj_name_list = env.get_obj_name_list()
    print('obj_name_list', obj_name_list)

    # %%
    env.add_scene_objects_to_moveit()

    camera_idx = env.scene.get_object_camera_idx('apple_0')

    env.move_joints_to(pos_list[int(camera_idx)])

    # %%
    grasp_pose = env.parse_adaptive_shape_grasp_pose('apple_0')

    # %%
    env.planning_scene.get_known_object_names()

    # %%
    env.objects

    # %%
    env.open_gripper()
    print("open--------------------------------------------------------------------------------")
    env.grasp(grasp_pose)


    # %%
    env.close_gripper()
    env.attach_object('apple_0')

    # %%
    place_pose = env.parse_place_pose('apple_0', 'plate_0')
    env.move_to_pose(place_pose)
    env.open_gripper()
    env.detach_object('apple_0')

    grasp_pose = env.parse_adaptive_shape_grasp_pose('plate_0')

    success = env.check(object_name='plate_0', pos_list=pos_list)
    return success

while success != True and iter_time < ITER_THRES:
    success = example()
    iter_time = iter_time + 1




