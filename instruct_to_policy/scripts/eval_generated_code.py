#!/usr/bin/env python3
import os 
import numpy as np
import argparse
from datetime import datetime
from src.lmp import *
from src.env.true_grounding_env import TrueGroundingEnv
from src.eval.evaluator import Evaluator
from src.config import cfg_tabletop
import rospy 
import rospkg

def filter_tasks(processed_data):
    """
    Filter the tasks that are not suitable for the environment based on hand-crafted rules.
    """
    # filter_funcs = [filter_drawer0]
    filter_funcs = []
    mask = np.ones((len(processed_data)))
    for func in filter_funcs:
        mask = func(processed_data, mask)
        
    filtered_processed_data = [processed_data[i] for i in range(len(processed_data)) if mask[i] == 1]
    return filtered_processed_data 
    


if __name__ == "__main__":
    
    # setup ros node
    rospy.init_node('eval_code', log_level=rospy.WARN)
    # get package root path 
    pkg_root = rospkg.RosPack().get_path('instruct_to_policy')

    # Get ROS parameters 
    world_name = rospy.get_param('~world_name', 'world_1_table_sort')
    # code_to_eval = rospy.get_param('~code_to_eval', 'generated_code_gpt3')
    # code_to_eval = rospy.get_param('~code_to_eval', 'generated_code_gpt3_few_shot')
    code_to_eval = rospy.get_param('~code_to_eval', 'generated_code_gpt3_few_shot_grasp_preference')

    # code_to_eval = rospy.get_param('~code_to_eval', 'generated_code_gpt4')
    # code_to_eval = rospy.get_param('~code_to_eval', 'generated_code_gpt4_few_shot')
    # code_to_eval = rospy.get_param('~code_to_eval', 'generated_code_gpt4_few_shot_grasp_preference')
    
    processed_file = os.path.join(pkg_root, f'data/benchmark/{code_to_eval}/processed_{world_name}.json')
    processed_file_path = os.path.join(pkg_root, processed_file)
    eval_items_file = os.path.join(pkg_root, f'data/benchmark/eval_items/{world_name}_eval_items.json')
    
    # log file should be appended with the formatted current time 
    time_str = datetime.now().strftime("%Y%m%d-%H:%M")
    log_file = rospy.get_param('~log_file', f'log/eval_log_{world_name}_{time_str}.log')
    log_file_path = os.path.join(pkg_root, log_file)
    # make log directory if not exist
    os.makedirs(os.path.dirname(log_file_path), exist_ok=True)
    
    # results file 
    result_file = rospy.get_param('~result_file', 
                                  os.path.join(pkg_root, f'data/benchmark/eval_results/{code_to_eval}/{world_name}_{time_str}.json'))
    result_file_path = os.path.join(pkg_root, result_file)
    # make result directory if not exist
    os.makedirs(os.path.dirname(result_file_path), exist_ok=True)
    
    # load generated code and eval items 
    with open(processed_file_path, 'r') as f:
        processed_data = json.load(f)
    with open(eval_items_file, 'r') as f:
        eval_items_list = json.load(f)

    eval_result_list = []

    # filter tasks that are not suitable for the environment
    # filtered_processed_data = filter_tasks(processed_data)
    for task_idx in range(0, len(processed_data)):
        # run code for each task
        data = processed_data[task_idx]
        eval_items_with_query = eval_items_list[task_idx]
        # if data is empty, append empty results to eval_result_list
        if len(data) == 0:
            eval_result_list.append({})
            continue
        query = data['query']
        code_str = data['code']
        assert query == eval_items_with_query['query'] # sanity check
        eval_items = eval_items_with_query['eval_items']
        
        rospy.loginfo("Running code for query: {}".format(query))
        rospy.loginfo("Code: \n'''{}\n'''".format(code_str))
        
        # setup environment and evaluator 
        env = TrueGroundingEnv(cfg_tabletop) 
        evaluator = Evaluator(env, log_file=log_file_path ,verbose=True, render=False)

        evaluator.run_eval(code_str, eval_items, query=query, repeat_times=5)
        results = evaluator.get_results()
        
        # append results to eval_result_list 
        eval_result_list.append(results)
    
    # write results to file
    with open(result_file_path, 'w') as f:
        json.dump(eval_result_list, f, indent=4)