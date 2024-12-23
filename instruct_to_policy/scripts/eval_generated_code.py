#!/usr/bin/env python3
import os
from cv2 import repeat 
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
    rospy.init_node('eval_code', log_level=rospy.DEBUG)
    # get current file path 
    
    
    # get package root path 
    pkg_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    os.chdir(pkg_root)
    
    # Get ROS parameters 
    world_name = rospy.get_param('~world_name', 'world_1_table_sort')
    code_dir = rospy.get_param('~code_dir', 'data/benchmark/generated_code')
    config_to_eval = rospy.get_param('~config_to_eval', 'text_few_shot_codellama')
    repeat_times = rospy.get_param('~repeat_times', 10)
    
    # ablation study toggles
    use_gt_2d_detections = rospy.get_param('~use_gt_2d_detections', False)
    use_gt_3d_bboxes = rospy.get_param('~use_gt_3d_bboxes', False)
    use_gt_planning_scene = rospy.get_param('~use_gt_planning_scene', False)
    
    config_file = os.path.join(pkg_root, f'scripts/src/configs/{config_to_eval}.yaml')
    cfg_tabletop = load_config(config_file)
    
    raw_file_path = os.path.join(pkg_root, code_dir, f'{config_to_eval}/raw_{world_name}.json')
    eval_items_file = os.path.join(pkg_root, f'data/benchmark/eval_items/{world_name}_eval_items.json')
    
    use_gt_perception = cfg_tabletop['perception']['use_ground_truth']
    
    # append ablation study toggles to config name
    output_config_to_eval = config_to_eval
    # setting multimodal env config from ros params
    if not use_gt_perception:
        
        if use_gt_2d_detections:
            output_config_to_eval += '_gt_2d_det'
            cfg_tabletop['grounding_model']['model_name'] = 'ground_truth'
        if use_gt_3d_bboxes:
            output_config_to_eval += '_gt_3d_bbox'
            cfg_tabletop['grounding_model']['model_name'] = 'ground_truth'
        if use_gt_planning_scene:
            output_config_to_eval += '_gt_plan_scene'
        
        # overwrite the config in the config dict 
        cfg_tabletop['perception']['use_gt_2d_detections'] = use_gt_2d_detections
        cfg_tabletop['perception']['use_gt_3d_bboxes'] = use_gt_3d_bboxes
        cfg_tabletop['perception']['use_gt_planning_scene'] = use_gt_planning_scene
    
    # log file should be appended with the formatted current time 
    time_str = datetime.now().strftime("%Y%m%d-%H:%M")
    log_file = rospy.get_param('~log_file', f'eval_{output_config_to_eval}_{world_name}_{time_str}.log')
    log_file_path = os.path.join(pkg_root, 'log', log_file)
    # make log directory if not exist
    os.makedirs(os.path.dirname(log_file_path), exist_ok=True)
    
    # results file 
    result_file = rospy.get_param('~result_file', 
                                  os.path.join(pkg_root, f'data/benchmark/eval_results/{output_config_to_eval}/{world_name}_{time_str}.json'))
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
        
        # assert query == eval_items_with_query['query'] # sanity check
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

        evaluator.run_eval(code_str, defined_functions, eval_items, query=query, repeat_times=repeat_times)
        results = evaluator.get_results()
        
        # append results to eval_result_list 
        eval_result_list.append(results)
        
        # TODO: Do we really need to delete there two objects?
        del env 
        del evaluator
    
    # write results to file
    with open(result_file_path, 'w') as f:
        json.dump(eval_result_list, f, indent=4)