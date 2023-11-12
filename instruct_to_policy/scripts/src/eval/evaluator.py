import numpy as np 
from typing import List, Dict, Tuple
import traceback
import logging 

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

    def init_metrics(self):
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
        self.metrics = {
            'grammer_correctness': [],
            'eval_items_results': []
        }
        
    def get_metrics(self):
        return self.metrics
        
    def reset(self):
        '''
        Reset the environment to the start state, and save the start state.
        '''
        self._reset_counter += 1
        self.logger.info("Resetting environment for the {}th time".format(self._reset_counter))
        
        self.env.reset()
        object_names = self.env.get_obj_name_list()
        self.env_start_state = {
            obj_name: {
                'pose': self.env.get_gt_obj_pose(obj_name),
                'bbox': self.env.get_3d_bbox(obj_name),
            } for obj_name in object_names
        }

    def run_eval(self, code_str: str, eval_items: List, repeat_times:int=1):
        '''
        Run the evaluation for the code snippet.
        '''
        
        self.init_metrics()
        
        # load vars 
        gvars, lvars = prepare_vars(self.env)
        
        for i in range(repeat_times):
            self.reset()
            try:
                self.logger.info("Running code for the {}th time".format(i+1))
                exec_safe(code_str, gvars, lvars)
            except Exception as e:
                # also record the traceback
                self.logger.error(f'Error when executing code for {i}-th trial: {e}')
                self.logger.error(traceback.format_exc())
                continue
            
            self.eval_env_state(eval_items)
            
        # log metrics in the end
        self.logger.info("\n######################## Metrics:\n {} \n###################################".format(self.metrics))
            

    def eval_env_state(self, eval_items: List):
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
        for eval_item in eval_items:
            eval_func = eval_func = getattr(self, eval_item['function'])
            eval_args = eval_item['args']
            eval_func(**eval_args)
      
    def check_relation_on(self, object_name:str, receptacle_name:str, **kwargs):
        '''
        Check if the object is on the receptacle by the spatial relationship of their bounding boxes.
        '''
        # intersection over object_bbox threshold
        iob_threshold = kwargs.get('iob_threshold', 0.8)
        
        # get object and receptacle bounding boxes
        # [x_min, y_min, z_min, x_max, y_max, z_max]
        object_bbox = self.env.get_3d_bbox(object_name)
        receptacle_bbox = self.env.get_3d_bbox(receptacle_name)
        
        # check if the object is on the receptacle by all the following conditions:
        # - if the object's z_max is higher than the receptacle's z_min
        # - if the object's xy bbox with the receptacle's xy bbox has an intersection over the object's xy bbox larger than a threshold
        is_on = object_bbox[5] > receptacle_bbox[2] # above the receptacle in z axis  
        object_bbox_xy = object_bbox[[0, 1, 3, 4]]
        receptacle_bbox_xy = receptacle_bbox[[0, 1, 3, 4]]
        iob = calc_2d_bbox_iob1(object_bbox_xy, receptacle_bbox_xy)
        is_on = is_on and iob > iob_threshold
        
        if self.verbose:
            print(f'Check if {object_name} is on {receptacle_name}: {is_on}')
        
        return is_on
         
       
    def check_relation_in(self, object_name:str, receptacle_name:str, **kwargs):
        '''
        Check if the object is in the receptacle by the spatial relationship of their bounding boxes.
        '''
        # intersection over object_bbox threshold
        iob_threshold = kwargs.get('iob_threshold', 0.8)
        
        # get object and receptacle bounding boxes
        # [x_min, y_min, z_min, x_max, y_max, z_max]
        object_bbox = self.env.get_3d_bbox(object_name)
        receptacle_bbox = self.env.get_3d_bbox(receptacle_name)
        
        # check if the object is on the receptacle by all the following conditions:
        # - if the object's 3d bbox with the receptacle's 3d bbox has an intersection over the object's 3d bbox larger than a threshold
        is_in = calc_3d_bbox_iob1(object_bbox, receptacle_bbox) > iob_threshold
        
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
            start_pose = self.get_start_obj_pose(object_name)
            desired_position = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
        else:
            desired_position = self.env.get_object_center_position(position_description)
            
        # calculate the distance between the object's current position and the desired position
        distance = np.linalg.norm(position - desired_position)
        is_close = distance < position_threshold
        
        if self.verbose:
            print(f'Check if {object_name} is close to {position_description}: {is_close}')
            
        return is_close
        
    def get_start_obj_pose(self, object_name:str):
        '''
        Get the object pose at the start of the environment.
        '''
        return self.env_start_state[object_name]['pose']