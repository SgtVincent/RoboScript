#!/usr/bin/env python3
import os 
import numpy as np
import argparse

from src.lmp import *
from src.env.simple_grounding_env import SimpleGroundingEnv
from src.config import cfg_tabletop
import rospy 
import rospkg

def filter_drawer0(processed_data, raw_data, mask):
    """
    Filter tasks involving interacting with drawer 0, since it is too far from the robot.
    """

    for i, data in enumerate(processed_data):
        if mask[i] == 0: # already filtered in previous filters 
            continue
        full_query = data['query']
        instruction = full_query.split(';')[-1]
        if 'drawer0' in instruction:
            mask[i] = 0
            
    return mask 


def filter_tasks(processed_data, raw_data):
    """
    Filter the tasks that are not suitable for the environment based on hand-crafted rules.
    """
    filter_funcs = [filter_drawer0]
    mask = np.ones((len(processed_data)))
    for func in filter_funcs:
        mask = func(processed_data, raw_data, mask)
        
    filtered_processed_data = [processed_data[i] for i in range(len(processed_data)) if mask[i] == 1]
    filtered_raw_data = [raw_data[i] for i in range(len(raw_data)) if mask[i] == 1]
    return filtered_processed_data, filtered_raw_data
    


if __name__ == "__main__":
    
    # setup ros node
    rospy.init_node('eval_code', log_level=rospy.WARN)
    # get package root path 
    pkg_root = rospkg.RosPack().get_path('instruct_to_policy')

    # Get ROS parameters 
    processed_file = rospy.get_param('~processed_file', 'data/generated_code/processed_table_cabinet_4.json')
    raw_file = rospy.get_param('~raw_file', 'data/generated_code/raw_table_cabinet_4.json')
    processed_file_path = os.path.join(pkg_root, processed_file)
    raw_file_path = os.path.join(pkg_root, raw_file)

    # setup environment
    env = SimpleGroundingEnv(cfg_tabletop)
    env.reset()

    # prepare variables for code execution 
    fixed_vars, variable_vars = prepare_vars(env)

    with open(processed_file_path, 'r') as f:
        processed_data = json.load(f)
    with open(raw_file_path, 'r') as f:
        raw_data = json.load(f)

    # filter tasks that are not suitable for the environment
    filtered_processed_data, filtered_raw_data = filter_tasks(processed_data, raw_data)
    
    # load code_str
    # for i, data in enumerate(filtered_processed_data):
    #     if i > 0:
    #         break
    data = filtered_processed_data[1]
    query = data['query']
    code_str = data['code']
    rospy.loginfo("Running code for query: {}".format(query))
    rospy.loginfo("Code: \n'''{}\n'''".format(code_str))
    exec_safe(code_str, fixed_vars, variable_vars)
    