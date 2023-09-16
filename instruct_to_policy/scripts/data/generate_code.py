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
# add parent directory to path
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# from src.prompt.message_definitions import *
from src.lmp import *
# from src.env.pybullet_env import PickPlaceEnv
from src.env.moveit_env import MoveitGazeboEnv
from src.config import cfg_tabletop
from src.openai_api_key import OPENAI_API_KEY
import rospy 

def load_queries(task_queries_file):
    """
    Load task queries from txt file. The first line is the world context, and the rest are task queries line by line:
    ''' 
    objects = [table, cabinet, cabinet.drawer0, cabinet.drawer1, cabinet.drawer2, cabinet.drawer3, panda_robot, round_cake_pan, black_bundt_pan, chips_can, mug, wine_glass]
    # open cabinet.drawer0
    # open cabinet.drawer1
    # open cabinet.drawer2
    ...
    '''
    """
    with open(task_queries_file, 'r') as f:
        lines = f.readlines()
    world_context = lines[0]
    task_queries = lines[1:]

    return world_context, task_queries

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


def parse_args():
    parser = argparse.ArgumentParser(description="Generate code for the robot arm manipulation.")
    parser.add_argument("--task-queries", type=str, default="data/task_queries/table_cabinet_0.txt",
                        help="Task queries file")
    parser.add_argument("--output-dir", type=str, default="data/generated_code",
                        help="Output directory (defaults to data/code)")
    parser.add_argument("--max-tokens", type=int, default=256,
                        help="Max tokens (defaults to 256)")
    
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
        dump_file=os.path.join(args.output_dir, os.path.basename(args.task_queries))
    )

    # load task queries
    world_context, task_queries = load_queries(args.task_queries)

    # generate one code snippet for each task query
    for i, task_query in enumerate(task_queries):
        if i>=10:
            break
        print(f"Generating code for task query {i}...")
        # generate code snippet
        lmp_tabletop_ui(task_query, world_context)
        lmp_tabletop_ui.clear_exec_hist()
        # save code snippet to file
        # output_file = os.path.join(args.output_dir, f"code_{i}.txt")
        # with open(output_file, 'w') as f:
        #     f.write(code_snippet)

