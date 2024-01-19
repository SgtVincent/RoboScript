#!/usr/bin/env python3
import os 
import numpy as np
import json
import argparse
from datetime import datetime
from src.lmp import *
from src.env.true_grounding_env import TrueGroundingEnv
from src.env.multimodal_env import MultiModalEnv
from src.eval.evaluator import Evaluator
from src.configs.config import load_config
import rospy

if __name__ == "__main__":
    
    # setup ros node
    rospy.init_node('eval_code', log_level=rospy.WARN)
    # get current file path 
    
    
    # get package root path 
    pkg_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    os.chdir(pkg_root)
    
    # Get ROS parameters 
    world_name = rospy.get_param('~world_name', 'world_1_table_sort')
    code_dir = rospy.get_param('~code_dir', 'data/benchmark/generated_code')
    config_to_eval = rospy.get_param('~config_to_eval', 'text_few_shot_codellama')
    config_file = os.path.join(pkg_root, f'scripts/src/configs/{config_to_eval}.yaml')
    cfg_tabletop = load_config(config_file)
    
    raw_file = os.path.join(pkg_root, code_dir, f'{config_to_eval}/raw_{world_name}.json')
    raw_file_path = os.path.join(pkg_root, raw_file)
    eval_items_file = os.path.join(pkg_root, f'data/benchmark/eval_items/{world_name}_eval_items.json')
    
    # log file should be appended with the formatted current time 
    time_str = datetime.now().strftime("%Y%m%d-%H:%M")
    log_file = rospy.get_param('~log_file', f'eval_{config_to_eval}_{world_name}_{time_str}.log')
    log_file_path = os.path.join(pkg_root, 'log', log_file)
    # make log directory if not exist
    os.makedirs(os.path.dirname(log_file_path), exist_ok=True)
    
    # results file 
    result_file = rospy.get_param('~result_file', 
                                  os.path.join(pkg_root, f'data/benchmark/eval_results/{config_to_eval}/{world_name}_{time_str}.json'))
    result_file_path = os.path.join(pkg_root, result_file)
    # make result directory if not exist
    os.makedirs(os.path.dirname(result_file_path), exist_ok=True)
    
    # load generated code and eval items 
    with open(raw_file_path, 'r') as f:
        raw_data = json.load(f)
    with open(eval_items_file, 'r') as f:
        eval_items_list = json.load(f)

    eval_result_list = []
    
    gt_perception = cfg_tabletop['perception']['use_ground_truth']

    for task_idx, data in enumerate(raw_data):
        # if task_idx not in [1]: # debug
        #     continue
        eval_items_with_query = eval_items_list[task_idx]
        # if data is empty, append empty results to eval_result_list
        if len(data) == 0:
            eval_result_list.append({})
            continue
        query = data['query']
        code_str = data['code_str']
        # from dict to list 
        defined_functions = [v for k, v in data['src_fs'].items()]
        
        assert query == eval_items_with_query['query'] # sanity check
        eval_items = eval_items_with_query['eval_items']
        
        rospy.loginfo("Running code for query: {}".format(query))
        rospy.loginfo("Code: \n'''{}\n'''".format(code_str))
        
        # setup environment and evaluator 
        if gt_perception:
            rospy.loginfo(f"Creating evaluation environment {TrueGroundingEnv.__name__}")
            env = TrueGroundingEnv(cfg_tabletop)
        else: 
            rospy.loginfo(f"Creating evaluation environment {MultiModalEnv.__name__}")
            env = MultiModalEnv(cfg_tabletop)

        evaluator = Evaluator(env, log_file=log_file_path ,verbose=True, render=False)

        evaluator.run_eval(code_str, defined_functions, eval_items, query=query, repeat_times=5)
        results = evaluator.get_results()
        
        # append results to eval_result_list 
        eval_result_list.append(results)
        
        # TODO: Do we really need to delete there two objects?
        del env 
        del evaluator
    
    # write results to file
    with open(result_file_path, 'w') as f:
        json.dump(eval_result_list, f, indent=4)