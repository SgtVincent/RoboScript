import numpy as np 
from typing import List, Dict, Tuple
import traceback
import logging 
import time 

from src.env.moveit_gazebo_env import MoveitGazeboEnv
from src.eval.utils import calc_2d_bbox_iob1, calc_3d_bbox_iob1, check_if_standing
from src.utils import exec_safe, prepare_vars

class Evaluator(object):
    '''
    Evaluate the performance of a code snippet against a set of evaluation items
    '''
    def __init__(self, env: MoveitGazeboEnv, log_file="./eval_log", verbose=False, render=False):
        self.env = env
        self.log_file = log_file
        self.verbose = verbose
        self.render = render
        
        self.env_start_state = None
        self._reset_counter = 0
        self.init_logger()
        
    def __del__(self):
        for handler in self.logger.handlers[:]:
            handler.close()
        
    def init_logger(self):
        '''
        Setup logger for the evaluator.
        '''
        # setup logger
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)  # Log all info and above

        # Create file handler which logs even debug messages
        fh = logging.FileHandler(self.log_file)
        fh.setLevel(logging.INFO)

        # Create console handler with a higher log level
        ch = logging.StreamHandler()
        ch.setLevel(logging.ERROR)

        # Create formatter and add it to the handlers
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)

        # Add the handlers to the logger
        self.logger.addHandler(fh)
        self.logger.addHandler(ch)

    def init_results_dict(self, query, repeat_times):
        '''
        Initialize the metrics for the evaluation items.
        
        Each eval item is a dictionary describing the evaluation function and arguments. 
        e.g. 
        {
            "function": "check_relation_on",
            "args": {
                "object_name": "apple",
                "receptacle_name": "white_ceramic_plate"
            }
        }
        '''
        # TODO: how to define metrics 
        self.results = {
            "query": query,
            "repeat_times": repeat_times,
        }
        
    def get_results(self):
        return self.results
        
    def reset(self):
        '''
        Reset the environment to the start state, and save the start state.
        '''
        self._reset_counter += 1
        self.logger.info("Resetting environment for the {}th time".format(self._reset_counter))
        
        self.env.reset()
        object_names = self.env.get_gazebo_model_names()
        self.env_start_state = {
            obj_name: {
                'pose': self.env.get_gt_obj_pose(obj_name),
                'bbox': self.env.get_gt_bbox(obj_name),
            } for obj_name in object_names
        }

    def run_eval(self, code_str: str, defined_functions: List[str], eval_items: List, query="", repeat_times:int=1):
        '''
        Run the evaluation for the code snippet.
        '''
        self.init_results_dict(query, repeat_times)
        
        for i in range(repeat_times):
            # reload vars 
            gvars, lvars = prepare_vars(self.env, defined_functions)
            # reset environment
            self.reset()
            exception = 0
            # try:
            #     self.logger.info("Running code for the {}th time".format(i+1))
            #     exec_safe(code_str, gvars, lvars)
            # except Exception as e:
            #     # also record the traceback
            #     exception = 1
            #     self.logger.error(f'Error when executing code for {i}-th trial: {e}')
            #     self.logger.error(traceback.format_exc())
            #     continue
            
            self.logger.info("Running code for the {}th time".format(i+1))
            exec_safe(code_str, gvars, lvars)
            
            # wait 3 seconds for the world state to change
            time.sleep(3)
            self.eval_env_state(i, eval_items, exception=exception)

        # log metrics in the end
        self.logger.info("\n######################## Results:\n {} \n###################################".format(self.results))
            

    def eval_env_state(self, repeat_idx, eval_items: List, exception):
        '''
        Evaluate the environment state according to eval_items.
        
        Each eval item is a dictionary describing the evaluation function and arguments. 
        e.g. 
        {
            "function": "check_relation_on",
            "args": {
                "object_name": "apple",
                "receptacle_name": "white_ceramic_plate"
            }
        }
        Then it should be executed as: 
        self.check_relation_on(object="apple", receptacle="white_ceramic_plate")
        '''
        self.results[repeat_idx] = {
            "grammer_correctness": 1-exception,
            "eval_items_results": [],
        }
        
        for eval_item in eval_items:
            eval_func = eval_func = getattr(self, eval_item['function'])
            eval_args = eval_item['args']
            result = int(eval_func(**eval_args))
            self.results[repeat_idx]['eval_items_results'].append(result)
      
      
    def check_relation_on(self, object_name:str, receptacle_name:str, **kwargs):
        '''
        Check if the object is on the receptacle by the spatial relationship of their bounding boxes.
        '''
        # intersection over object_bbox threshold
        iob_threshold = kwargs.get('iob_threshold', 0.8)
        flag_not = kwargs.get('flag_not', 0)
        
        # get object and receptacle bounding boxes
        # [x_min, y_min, z_min, x_max, y_max, z_max]
        # NOTE: for mutlimodal env, should use get_gt_bbox
        object_bbox_center, object_bbox_size = self.env.get_gt_bbox(object_name)
        receptacle_bbox_center, receptacle_bbox_size = self.env.get_gt_bbox(receptacle_name)
        object_bbox = np.concatenate([object_bbox_center - object_bbox_size/2, object_bbox_center + object_bbox_size/2])
        receptacle_bbox = np.concatenate([receptacle_bbox_center - receptacle_bbox_size/2, receptacle_bbox_center + receptacle_bbox_size/2])
        
        # check if the object is on the receptacle by all the following conditions:
        # - if the object's z_max is higher than the receptacle's z_min
        # - if the object's xy bbox with the receptacle's xy bbox has an intersection over the object's xy bbox larger than a threshold
        is_on = object_bbox[5] > receptacle_bbox[2] # above the receptacle in z axis  
        object_bbox_xy = object_bbox[[0, 1, 3, 4]]
        receptacle_bbox_xy = receptacle_bbox[[0, 1, 3, 4]]
        iob = calc_2d_bbox_iob1(object_bbox_xy, receptacle_bbox_xy)
        is_on = is_on and iob > iob_threshold
        
        if flag_not:
            is_not_on = not is_on
            if self.verbose:
                print(f'Check if {object_name} is not on {receptacle_name}: {is_not_on}')
            return is_not_on
        else:
            if self.verbose:
                print(f'Check if {object_name} is on {receptacle_name}: {is_on}')
            
            return is_on
         
       
    def check_relation_in(self, object_name:str, receptacle_name:str, **kwargs):
        '''
        Check if the object is in the receptacle by the spatial relationship of their bounding boxes.
        '''
        # intersection over object_bbox threshold
        iob_threshold = kwargs.get('iob_threshold', 0.8)
        flag_not = kwargs.get('flag_not', 0)
        
        # get object and receptacle bounding boxes
        # [x_min, y_min, z_min, x_max, y_max, z_max]
        # NOTE: for mutlimodal env, should use get_gt_bbox
        object_bbox_center, object_bbox_size = self.env.get_gt_bbox(object_name)
        receptacle_bbox_center, receptacle_bbox_size = self.env.get_gt_bbox(receptacle_name)
        object_bbox = np.concatenate([object_bbox_center - object_bbox_size/2, object_bbox_center + object_bbox_size/2])
        receptacle_bbox = np.concatenate([receptacle_bbox_center - receptacle_bbox_size/2, receptacle_bbox_center + receptacle_bbox_size/2])
        
        # check if the object is on the receptacle by all the following conditions:
        # - if the object's 3d bbox with the receptacle's 3d bbox has an intersection over the object's 3d bbox larger than a threshold
        is_in = calc_3d_bbox_iob1(object_bbox, receptacle_bbox) > iob_threshold
        
        if flag_not:
            is_not_in = not is_in
            if self.verbose:
                print(f'Check if {object_name} is not in {receptacle_name}: {is_not_in}')
            return is_not_in
        else:
            if self.verbose:
                print(f'Check if {object_name} is in {receptacle_name}: {is_in}')
                
            return is_in
        
    
    def check_object_pose(self, object_name:str, pose_description:str, **kwargs):
        '''
        Check if the object is at the correct pose. 
        '''
        # get object pose
        object_pose = self.env.get_gt_obj_pose(object_name)
        position = np.array([object_pose.position.x, object_pose.position.y, object_pose.position.z])
        orientation = np.array([object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w])
        
        # TODO: how should we define flexible and scalable pose checking functions?
        if pose_description == "standing":
            is_standing = check_if_standing(orientation)
            
            if self.verbose:
                print(f'Check if {object_name} is standing: {is_standing}')
            
            return is_standing
            
        else: 
            raise NotImplementedError(f"Pose description {pose_description} is not implemented")
    
    def check_object_position(self, object_name:str, position_description:str, **kwargs):
        '''
        Check if the object is at the correct position. 
        '''
        # position threshold
        position_threshold = kwargs.get('position_threshold', 0.1)
        
        # get object current position
        object_pose = self.env.get_gt_obj_pose(object_name)
        position = np.array([object_pose.position.x, object_pose.position.y, object_pose.position.z])
        
        # get desired position
        if position_description.startswith('start_position:'):
            start_object_name = position_description.split(':')[1].strip()
            start_pose = self.get_start_obj_pose(start_object_name)
            desired_position = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
        else:
            desired_position = self.env.get_object_center_position(position_description)
            
        # calculate the distance between the object's current position and the desired position
        distance = np.linalg.norm(position - desired_position)
        is_close = distance < position_threshold
        
        if self.verbose:
            print(f'Check if {object_name} is close to {position_description}: {is_close}')
            
        return is_close
        
    def check_joint_state(self, joint_name:str, **kwargs):
        '''
        Check if the joint is at the correct state. 
        '''
        check_position_func = kwargs.get('check_position_func', lambda x: x)
        
        # get joint properties from gazebo 
        joint_name = joint_name.replace('.', '::')
        joint_properties = self.env.get_joint_properties(joint_name)
        joint_type = joint_properties.type
        joint_position = joint_properties.position[0]
        
        return check_position_func(joint_position)
        
        
    def get_start_obj_pose(self, object_name:str):
        '''
        Get the object pose at the start of the environment.
        '''
        return self.env_start_state[object_name]['pose']