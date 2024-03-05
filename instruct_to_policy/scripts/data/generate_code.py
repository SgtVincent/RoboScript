#!/usr/bin/env python3
"""
This script is used to generate the code for the robot arm manipulation.
"""

import os 
import traceback
import numpy as np
import openai
import argparse 
import shapely
import re
import json
# add parent directory to path
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# from src.prompt.message_definitions import *
from src.lmp import *
from src.configs.config import load_config
import rospy 

def load_queries(task_queries_file):
    """
    Load task queries from txt file. The first line is the world context, and the rest are task queries line by line:
    ''' 
    
    objects = [table, cabinet, cabinet.drawer_0, cabinet.drawer_1, cabinet.drawer_2, cabinet.drawer_3, panda_robot] ; # open cabinet.drawer_0
    ...
    '''
    """
    with open(task_queries_file, 'r') as f:
        lines = f.readlines()
    
    # use regex to extract the query in each line:
    # objects = [table, cabinet, cabinet.drawer_0, cabinet.drawer_1, cabinet.drawer_2, cabinet.drawer_3, panda_robot] ; # open cabinet.drawer_0

    valid_line_pattern = re.compile(r'(?P<context>objects.*);\s*#(?P<query>.*)')
    task_queries = []
    for line in lines:
        match = valid_line_pattern.match(line)
        if match:
            context = match.group('context')
            query = match.group('query')
            task_query = context + "; #" + query
            task_queries.append(task_query)

    return task_queries

def prepare_vars_detached():

    """Prepare variables including APIs and objects for LMPs """

    fixed_vars = {"np": np, "rospy": rospy}
    # fixed_vars.update(
    #     {name: eval(name) for name in shapely.geometry.__all__ + shapely.affinity.__all__}
    # )

    variable_vars = {
        k: None
        for k in [
            "move_group",
            "PoseStamped",
            "Pose",
            "Point",
            "Quaternion",
            "get_object_center_position",
            "get_object_pose",
            "get_3d_bbox",
            "get_obj_name_list",
            "detect_objects",
            "get_object_joint_info",
            "parse_adaptive_shape_grasp_pose",
            "parse_horizontal_grasp_pose",
            "parse_place_pose",
            "open_gripper",
            "close_gripper",
            "attach_object",
            "detach_object",
            "move_to_pose",
            "follow_path",
            "get_gripper_pose",
            "grasp",
        ]
    }
    variable_vars["say"] = lambda msg: print(f"robot says: {msg}")
    
    # add moveit interfaces to variables
    variable_vars.update(
        {
            k: None
            for k in [
                "move_group",
                "gripper_group",
            ]
        }
    )
    return fixed_vars, variable_vars

def process_raw_output(raw_path, processed_path):
    """
    Convert raw output json to {query: code} pairs

    Raw output json:
    [{
        "context": context,
        "query": use_query,
        "src_fs": src_fs,
        "code_str": code_str,
        "gvars": list(gvars.keys()),
        "lvars": list(lvars.keys()),
    },
    ...
    ]
    """
    with open(raw_path, 'r') as f:
        raw_data = json.load(f)
    
    processed_data = []
    for data in raw_data:
        # if data is empty, append empty dict
        if len(data) == 0:
            processed_data.append({})
            continue 
        context = data['context']
        query = data['query']
        query = context + query

        src_fs = data['src_fs']
        code = data['code_str']
        if len(src_fs) > 0:
            fs_definition_str = '\n'.join([v for k, v in src_fs.items()])
            code = fs_definition_str + '\n' + code
        
        processed_data.append({
            "query": query,
            "code": code
        })

    with open(processed_path, 'w') as f:
        json.dump(processed_data, f, indent=4)



def parse_args():
    parser = argparse.ArgumentParser(description="Generate code for the robot arm manipulation.")
    # parser.add_argument("--task-queries", type=str, default="data/table_cabinet_100/task_queries/table_cabinet_0.txt",
    #                     help="Task queries file")
    # parser.add_argument("--output-dir", type=str, default="data/table_cabinet_100/generated_code",
    #                     help="Output directory")
    # parser.add_argument("--max-tokens", type=int, default=2048,
    #                     help="Max tokens (defaults to 2048)")
    # parser.add_argument("--max-queries", type=int, default=200, 
    #                     help="Max number of task queries to generate (defaults to 200)")
    parser.add_argument("--config_file", type=str, default="perception_few_shot_gpt_3.yaml")
    parser.add_argument("--task-queries", type=str, default="data/benchmark/task_queries/world_1_table_sort.txt",
                        help="Task queries file")
    parser.add_argument("--output-dir", type=str, default="data/benchmark/generated_code",
                        help="Output directory (defaults to data/code)")
    parser.add_argument("--max-tokens", type=int, default=4096,
                        help="Max tokens (defaults to 4096)")
    parser.add_argument("--max-queries", type=int, default=10, 
                        help="Max number of task queries to generate (defaults to 200)")
    # change to instruct_to_policy directory
    dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    os.chdir(dir)
    
    args, unknown_args = parser.parse_known_args()

    # create a subdir for this config 
    args.output_dir = os.path.join(args.output_dir, args.config_file.split('.')[0])
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
    
    print(f"Saving generated code and logs to the output dir: {args.output_dir}")
    
    return args



if __name__ == "__main__":

    # parse arguments
    args = parse_args()
    
    # Initialize LMP instances
    cfg_tabletop = load_config(args.config_file)

    # prepare vars including APIs and constants
    fixed_vars, variable_vars = prepare_vars_detached()

    # load task queries
    task_queries = load_queries(args.task_queries)

    # if no valid task queries, exit
    if len(task_queries) == 0:
        print(f"No valid task queries in {args.task_queries}")
        exit()

    exception_log = ""
    dump_hist_list = []
    # generate one code snippet for each task query
    for i, task_query in enumerate(task_queries):
        if i >= args.max_queries:
            break
        # if i not in [4]:
        #     continue
        try:
            # remove extra '#' and '\n' in query line
            # task_query = task_query.replace('#', '').replace('\n', '')
            lmp_tabletop_ui = setup_LMP(None, cfg_tabletop, debug_mode=True, detached_mode=True)

            print(f"Generating code for task query {i}...")
            # generate code snippet
            lmp_tabletop_ui(task_query, "")
            # append dump_hist to list 
            dump_hist_list.append(lmp_tabletop_ui.dump_hist[-1])
            
            # delete lmp_tabletop_ui to release memory/ kill threads
            del lmp_tabletop_ui
            
        except Exception as e:
            # raise e 
            exception_log += "----------\n"
            exception_log += f"Cannot generate code for task query {i}: {task_query} \n"
            exception_log += f"Exception: {e} \n"
            exception_log += f"Traceback: {traceback.format_exc()} \n"
            exception_log += "----------\n"
            # add empty history to dump_hist
            dump_hist_list.append({})
            
    # write exception log to file
    exception_log_file = os.path.basename(args.task_queries).replace('.txt', '_exception_log.txt')
    exception_log_path = os.path.join(args.output_dir, exception_log_file)
    with open(exception_log_path, 'w') as f:
        f.write(exception_log)

    # save generated code snippets to json 
    raw_output_file = "raw_" + os.path.basename(args.task_queries).replace('.txt', '.json')
    raw_output_path = os.path.join(args.output_dir, raw_output_file)
    with open(raw_output_path, "w") as f:
        json.dump(dump_hist_list, f, indent=4)

    # convert raw output json to {query: code} pairs
    # raw_output_file = "raw_" + os.path.basename(args.task_queries).replace('.txt', '.json')
    # raw_output_path = os.path.join(args.output_dir, raw_output_file)

    # NOTE: not used anymore since defined functions and main body should be executed separately    
    # processed_file = "processed_" + os.path.basename(args.task_queries).replace('.txt', '.json')
    # processed_path = os.path.join(args.output_dir, processed_file)
    # process_raw_output(raw_output_path, processed_path)

