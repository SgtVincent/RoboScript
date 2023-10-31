"""
This script is used to generate the code for the robot arm manipulation.
The code is generated with OpenAI API (gpt-3.5-turbo) chat completion.
"""

#!/usr/bin/env python3
import os 
import numpy as np
import openai
import argparse 
import shapely
import re
# add parent directory to path
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# from src.prompt.message_definitions import *
from src.lmp import *
# from src.env.pybullet_env import PickPlaceEnv
from src.env.moveit_gazebo_env import MoveitGazeboEnv
from src.config import cfg_tabletop
from src.openai_api_key import OPENAI_API_KEY
import rospy 

def load_queries(task_queries_file):
    """
    Load task queries from txt file. The first line is the world context, and the rest are task queries line by line:
    ''' 
    
    objects = [table, cabinet, cabinet.drawer0, cabinet.drawer1, cabinet.drawer2, cabinet.drawer3, panda_robot] ; # open cabinet.drawer0
    ...
    '''
    """
    with open(task_queries_file, 'r') as f:
        lines = f.readlines()
    
    # use regex to extract the query in each line:
    # objects = [table, cabinet, cabinet.drawer0, cabinet.drawer1, cabinet.drawer2, cabinet.drawer3, panda_robot] ; # open cabinet.drawer0

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

    fixed_vars = {"np": np}
    fixed_vars.update(
        {name: eval(name) for name in shapely.geometry.__all__ + shapely.affinity.__all__}
    )
    variable_vars = {
        k: None
        for k in [
            "get_bbox",
            "get_obj_pos",
            "get_color",
            "is_obj_visible",
            "denormalize_xy",
            "get_obj_names",
            "get_corner_name",
            "get_side_name",
            "get_ee_pose",
            "parse_pose",
            "open_gripper",
            "close_gripper",
            "move_to_pose",
            "move_joints_to",
            "add_object_to_scene",
            "attach_object",
            "detach_object"
            # DO NOT use mid-level skills?
            # "grasp",
            # "place"
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
    parser.add_argument("--task-queries", type=str, default="data/task_queries/table_cabinet_0.txt",
                        help="Task queries file")
    parser.add_argument("--output-dir", type=str, default="data/generated_code",
                        help="Output directory (defaults to data/code)")
    parser.add_argument("--max-tokens", type=int, default=256,
                        help="Max tokens (defaults to 256)")
    parser.add_argument("--max_queries", type=int, default=200, 
                        help="Max number of task queries to generate (defaults to 200)")
    args = parser.parse_args()

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    return args



if __name__ == "__main__":

    # parse arguments
    args = parse_args()

    # set openai api key
    openai.api_key = OPENAI_API_KEY

    # Initialize LMP instances
    cfg_tabletop = copy.deepcopy(cfg_tabletop)
    cfg_tabletop["env"] = dict()
    # TODO: load table top from simulation 
    cfg_tabletop["env"]["coords"] = lmp_tabletop_coords

    # prepare vars including APIs and constants
    fixed_vars, variable_vars = prepare_vars_detached()

    # creating the function-generating LMP
    lmp_fgen = LMPFGen(cfg_tabletop["lmps"]["fgen"], fixed_vars, variable_vars)

    # creating other low-level LMPs
    variable_vars.update(
        {
            k: LMP(k, cfg_tabletop["lmps"][k], lmp_fgen, fixed_vars, variable_vars)
            for k in ["parse_obj_name", "parse_question", "transform_shape_pts"]
        }
    )

    # creating the LMP that deals w/ high-level language commands
    cfg_tabletop["lmps"]["tabletop_ui"]["debug_mode"] = True
    lmp_tabletop_ui = LMP(
        "tabletop_ui", cfg_tabletop["lmps"]["tabletop_ui"], lmp_fgen, fixed_vars, variable_vars,
    )

    # load task queries
    task_queries = load_queries(args.task_queries)

    # if no valid task queries, exit
    if len(task_queries) == 0:
        print(f"No valid task queries in {args.task_queries}")
        exit()

    exception_log = ""
    # generate one code snippet for each task query
    for i, task_query in enumerate(task_queries):
        if i >= args.max_queries:
            break
        try:
            # remove extra '#' and '\n' in query line
            task_query = task_query.replace('#', '').replace('\n', '')

            print(f"Generating code for task query {i}...")
            # generate code snippet
            lmp_tabletop_ui(task_query, "")
            lmp_tabletop_ui.clear_exec_hist()
        except Exception as e:
            exception_log += "----------\n"
            exception_log += f"Cannot generate code for task query {i}: {task_query} \n"
            exception_log += f"Exception: {e} \n"
            exception_log += "----------\n"
            
    # write exception log to file
    exception_log_file = os.path.basename(args.task_queries).replace('.txt', '_exception_log.txt')
    exception_log_path = os.path.join(args.output_dir, exception_log_file)
    with open(exception_log_path, 'w') as f:
        f.write(exception_log)

    # save generated code snippets to json 
    raw_output_file = "raw_" + os.path.basename(args.task_queries).replace('.txt', '.json')
    raw_output_path = os.path.join(args.output_dir, raw_output_file)
    lmp_tabletop_ui.save_dump_hist(raw_output_path)

    # convert raw output json to {query: code} pairs
    # raw_output_file = "raw_" + os.path.basename(args.task_queries).replace('.txt', '.json')
    # raw_output_path = os.path.join(args.output_dir, raw_output_file)
    
    processed_file = "processed_" + os.path.basename(args.task_queries).replace('.txt', '.json')
    processed_path = os.path.join(args.output_dir, processed_file)
    process_raw_output(raw_output_path, processed_path)

