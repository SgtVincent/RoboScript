#!/usr/bin/env python3
import os 
import numpy as np
import openai
import cv2
import argparse

from src.lmp import *
from src.env.simple_grounding_env import SimpleGroundingEnv
from src.config import cfg_tabletop
import rospy 

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
    


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--processed_file", type=str, default="data/generated_code/processed_table_cabinet_4.json",
        help=r"File of processed {query: code} pairs to check"
    )
    parser.add_argument(
        "--raw_file", type=str, default="data/generated_code/raw_table_cabinet_4.json",
        help="File of raw code generation result, used for importing pre-defined variables"
    )

    args = parser.parse_args()
    return args

if __name__ == "__main__":
    
    args = parse_args()
    
    # setup env and LMP
    rospy.init_node('eval_code', anonymous=True, log_level=rospy.WARN)
    env = SimpleGroundingEnv(cfg_tabletop)
    env.reset()

    # prepare variables for code execution 
    fixed_vars, variable_vars = prepare_vars(env)

    with open(args.processed_file, 'r') as f:
        processed_data = json.load(f)
    with open(args.raw_file, 'r') as f:
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
    