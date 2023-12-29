# %%
import os 
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R 
import rospy 
import rospkg
import traceback
# import jupyros as jr
import moveit_commander
from std_srvs.srv import Empty
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
# [-0.6666276559536917, -0.9423825665605631, 0.0187059330066652, -2.4728545562580195, -0.10638790550563826, 2.0371070687540453, 0.9745453378439838],
# [-0.001209562553452295, -0.7798870970324466, -0.0025529672049247384, -2.3749749452691327, -0.0031800321414839528, 1.5748067867097109, 0.7786260150587473]
# [-0.001209562553452295, -0.7798870970324466, -0.0025529672049247384, -2.3749749452691327, -0.0031800321414839528, 1.5748067867097109, 0.7786260150587473]
# 0.5889080213203765, -0.2942316264692174, -0.35443429826853545, -1.0650221232029426, -0.007263506037907467, 1.3009005137419631, 0.9725401304339114
ITER_THRES = 2
DETECT_FIRST = True
PLANNING = True
LOAD=False
# pos_list = [[0.9948088120117522, -1.116624177147722, -1.0540387755777176, -1.4079695810380404, 0.3141663355277449, 1.7899097791506897, -0.23147634689511404]]
# [-0.019717047784673536, 0.20466893432184957, -0.16597191178380394, -1.357592204462018, 0.22251903023984695, 1.320185246388116, 0.11779099716307696],
pos_list = [[0.8562231370065547, -1.2388686503967279, -0.6676529594031755, -1.9809160322430088, -0.13393191242259475, 1.7970343970457712, 0.2586779892254231]]

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

def construct_queries(obj_name_list, query):
    context = 'object = ['
    for i, obj_name in enumerate(obj_name_list):
        context = context + obj_name
        if i != len(obj_name_list) - 1:
            context = context + ', '
        else:
            context = context + ']; # '
    context = context + query
    return context

def clear_octomap():
    # rospy.init_node('clear_octomap_script')
    rospy.wait_for_service('/clear_octomap')
    try:
        # 创建服务调用者并调用服务
        clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        clear_octomap()
        rospy.loginfo("Octomap cleared.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def construct_query(query, is_vertical):
    if 'open' in query:
        if is_vertical:
            query = query + ' with parse_vertical_grasp_pose, generate_arc_path_around_joint and follow_path'
        else:
            query = query + ' with horizontal_grasp_pose, moving_in_direction'
    elif 'close' in query:
        if is_vertical:
            query = query + ' with parse_vertical_grasp_pose, generate_arc_path_around_joint and follow_path'
        else:
            query = query + ' with horizontal_grasp_pose, moving_in_direction'
    return query


def example(detect_first=False):
    # for i, reset_pos in enumerate(reset_pos_list):
    #     env.move_joints_to(reset_pos)
    #     if i == 0:
    #         image_dict = env.get_sensor_data()
    #     else:
    #         image_set = env.get_sensor_data()
    #         for key, value in image_dict.items(): 
    #             value.extend(image_set[key])
    #             image_dict[key] = value

    # query = 'open the drawer and put the toy in it then close the drawer'
    # all_object_list = ["drawer","handle","toy"]
    all_object_list =  ["apple", "drawer", "milk", "handle"]
    data = {
            'pos_list': pos_list
        }
    
    env.scene.update_pos_list(data)
    # # %%
    # print(os.getcwd())

    # TODO: object_list should be filtered

    # # %%
    if detect_first == True:
        plan_object_list = env.detect_objects_with_moving(object_list=all_object_list, planning=PLANNING, is_load=LOAD)

        # # %%
        # env.scene.visualize_3d_bboxes()
        object_list = env.get_obj_name_list()
        print('detected_object_list', object_list)

        if 'handle_0' in object_list:
            is_vertical = env.is_handle_vertical(env.scene.bbox_oriented_3d_dict["handle_0"])
        else:
            is_vertical = None


    # %%
    # env.add_scene_objects_to_moveit()

    if plan_object_list is not None:
        for query_objects in plan_object_list:
            query = query_objects['query']
            object_list = query_objects['objects']

            print('query', query, 'object_list', object_list)

            construct_query(query, is_vertical=is_vertical)

            env.detect_objects_with_moving(object_list=object_list)

            object_list = env.get_obj_name_list()

            env.generate_exec_code(object_list, query, cfg_tabletop)

            env.execute_code(detect_first=detect_first)
    else:
        query = construct_query(query, is_vertical=is_vertical)

        object_list = env.get_obj_name_list()

        # env.generate_exec_code(object_list, query, cfg_tabletop)

        env.execute_code(detect_first=detect_first)

    # camera_idx = env.scene.get_object_camera_idx('apple_0')

    # env.move_joints_to(pos_list[int(camera_idx)])

    # # %%
    # grasp_pose = env.parse_adaptive_shape_grasp_pose('apple_0')

    # # %%
    # env.planning_scene.get_known_object_names()

    # # %%
    # env.objects

    # # %%
    # env.open_gripper()
    # print("open--------------------------------------------------------------------------------")
    # env.grasp(grasp_pose)


    # # %%
    # env.close_gripper()
    # env.attach_object('apple_0')

    # # %%
    # place_pose = env.parse_place_pose('apple_0', 'plate_0')
    # env.move_to_pose(place_pose)
    # env.open_gripper()
    # env.detach_object('apple_0')

    # grasp_pose = env.parse_adaptive_shape_grasp_pose('plate_0')

    # success = env.check(object_name='plate_0', pos_list=pos_list, task_query=query)
    return success

if __name__ == '__main__':
    clear_octomap()
    scene = moveit_commander.PlanningSceneInterface()
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -0.38  # above the panda_hand frame
    box_pose.pose.position.z = 1  # above the panda_hand frame

    box_name = "box1"
    scene.add_box(box_name, box_pose, size=(0.1, 0.5, 2))
    scene.attach_box('panda_link0', box_name)


    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -0.33  # above the panda_hand frame
    box_pose.pose.position.y = -0.7  # above the panda_hand frame
    box_pose.pose.position.z = 0.2  # above the panda_hand frame

    box_name = "box2"
    scene.add_box(box_name, box_pose, size=(0.9, 0.4, 0.4))
    scene.attach_box('panda_link0', box_name)



    success = example(detect_first=DETECT_FIRST)




