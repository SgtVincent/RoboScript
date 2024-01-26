#! /usr/bin/env python
import os
from pdb import run
import sys
import rospy
import argparse
import time
import json
import traceback
from datetime import datetime
import random
import numpy as np
import torch
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Point, Quaternion

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# from src.prompt.message_definitions import *
from src.lmp import *
from src.configs.config import load_config
from src.env.true_grounding_env import TrueGroundingEnv
from src.eval.evaluator import Evaluator

def get_models():
    """
    Returns a list of models in the models directory
    """
    package_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ycb_metadata_file = os.path.join(package_root, 'data/ycb/metadata.json')
    google_metadata_file = os.path.join(package_root, 'data/google_scanned_object/object_metadata.json')
    
    # Load ycb metadata
    ycb_metadata = json.load(open(ycb_metadata_file, 'r'))
    for model_id in ycb_metadata['objects'].keys():
        model_name = ycb_metadata['objects'][model_id]['model_name']
        ycb_metadata['objects'][model_id]['sdf_path'] = os.path.join(package_root, 'data/ycb/models', model_id, f'{model_name}.sdf')
    
    # Load google scanned objects metadata
    google_metadata = json.load(open(google_metadata_file, 'r'))
    for model_id in google_metadata['objects'].keys():
        model_name = google_metadata['objects'][model_id]['model_name']
        google_metadata['objects'][model_id]['sdf_path'] = os.path.join(package_root, 'data/google_scanned_object/models', model_id, f'{model_id}.sdf')
    
    # merge metadata
    metadata = {'objects': {}}
    # for model_id in ycb_metadata['objects'].keys():
    #     metadata['objects'][model_id] = ycb_metadata['objects'][model_id]
    for model_id in google_metadata['objects'].keys():
        metadata['objects'][model_id] = google_metadata['objects'][model_id]
    
    return metadata

def spawn_model(model_name, model_sdf_file, model_pose, reference_frame="world"):
    """
    Spawn model in Gazebo
    """
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        model_xml = open(model_sdf_file, 'r').read()
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        req = SpawnModelRequest()
        req.model_name = model_name
        req.model_xml = model_xml
        req.initial_pose = model_pose
        req.reference_frame = reference_frame
        spawn_model_prox(req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def delete_model(model_name):
    """
    Delete model in Gazebo
    """
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        req = DeleteModelRequest()
        req.model_name = model_name
        delete_model_prox(req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def run_pick_and_place_eval(evaluator: Evaluator, object_name, object_file_path, object_pose, 
                            receptable_name="white_ceramic_plate", repeat_times=10):
    """
    Run pick and place evaluation
    """      
      
    evaluator.init_results_dict(f"pick_and_place_{object_name}", repeat_times)
        
    for i in range(repeat_times):
        # reset environment
        evaluator.reset()
        exception = 0
        try:
            
            # pause_physics_prox()
            spawn_model(object_name, object_file_path, object_pose)
            rospy.sleep(1.0)
            # unpause_physics_prox()
            
            evaluator.logger.info("Running code for the {}th time".format(i+1))
            
            # pick and place code
            env = evaluator.env
            # Grasp the object
            env.open_gripper()
            grasp_pose = env.parse_adaptive_shape_grasp_pose(object_name)
            env.grasp(grasp_pose)
            env.close_gripper()
            env.attach_object(object_name)

            # Move the object to the receptacle
            place_pose = env.parse_place_pose(object_name, receptable_name)
            env.move_to_pose(place_pose)

            # Release the object
            env.open_gripper()
            env.detach_object(object_name)
            
        except Exception as e:
            # also record the traceback
            exception = 1
            evaluator.logger.error(f'Error when executing code for {i}-th trial: {e}')
            evaluator.logger.error(traceback.format_exc())
            continue
        
        # wait 3 seconds for the world state to change
        time.sleep(3)
        
        eval_items = [{
            "function": "check_relation_on",
            "args": {
                "object_name": object_name,
                "receptacle_name": receptable_name
            }
        }]
        evaluator.eval_env_state(i, eval_items, exception=exception)

        delete_model(object_name)

    # log metrics in the end
    evaluator.logger.info("\n######################## Results:\n {} \n###################################".format(evaluator.results))

def parse_args():
    """
    Parse arguments
    """
    parser = argparse.ArgumentParser(description='Instruct to Policy')
    parser.add_argument('--config_to_eval', default='text_gpt_4')
    parser.add_argument('--x', type=float, default=-0.10, help='x coordinate of the object')
    parser.add_argument('--y', type=float, default=-0.25, help='y coordinate of the object')
    parser.add_argument('--table_height', type=float, default=0.815, help='z coordinate of the table top')
    parser.add_argument('--random_seed', default=42, help='answer to life the universe and everything')
    args = parser.parse_args([])
    
    # set random seed 
    random.seed(args.random_seed)
    np.random.seed(args.random_seed)
    torch.cuda.manual_seed(args.random_seed)
    
    return args

if __name__ == '__main__':

    # Initialize ROS node
    rospy.init_node('test_pick_and_place', anonymous=False)
    args = parse_args()
    
    # get models from metadata 
    metadata = get_models()

    # Initialize Envrionment and connect to gazebo 
    pkg_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    config_file = os.path.join(pkg_root, f'scripts/src/configs/{args.config_to_eval}.yaml')
    cfg_tabletop = load_config(config_file)
    
    env = TrueGroundingEnv(cfg_tabletop)
    
    # log file should be appended with the formatted current time 
    time_str = datetime.now().strftime("%Y%m%d-%H:%M")
    log_file = rospy.get_param('~log_file', f'eval_pick_and_place.log')
    log_file_path = os.path.join(pkg_root, 'log', log_file)
    # make log directory if not exist
    os.makedirs(os.path.dirname(log_file_path), exist_ok=True)
    
    # results file 
    result_file_path = os.path.join(pkg_root, f'data/benchmark/eval_results/pick_and_place_{time_str}.json')
    
    # make result directory if not exist
    os.makedirs(os.path.dirname(result_file_path), exist_ok=True)
    eval_result_list = []

    # Service to spawn models in Gazebo
    # spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    # delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    pause_physics_prox = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpause_physics_prox = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

    # Iterate through object models
    for object_id in metadata['objects'].keys():
        # Spawn object model in Gazebo
        object_file_path = metadata['objects'][object_id]['sdf_path']
        object_bbox_center = metadata['objects'][object_id]['bbox_center']
        object_bbox_size = metadata['objects'][object_id]['bbox_size']
        object_z_margin = object_bbox_size[2] / 2.0 - object_bbox_center[2] + 0.01
        object_pose = Pose(
            position=Point(*[args.x, args.y, args.table_height + object_z_margin]),
            orientation=Quaternion(*[0, 0, 0, 1])
        )
        object_name = metadata['objects'][object_id]['model_name']

        # Run pick and place evaluation
        evaluator = Evaluator(env, log_file=log_file_path ,verbose=True, render=False)
        run_pick_and_place_eval(evaluator, object_name, object_file_path, object_pose, repeat_times=10)
        results = evaluator.get_results()
        
        # append results to eval_result_list 
        eval_result_list.append(results)
        del evaluator
    
    # write results to file
    with open(result_file_path, 'w') as f:
        json.dump(eval_result_list, f, indent=4)