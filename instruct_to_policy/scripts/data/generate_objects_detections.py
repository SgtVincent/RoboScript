#!/usr/bin/env python3
"""
This script is used to generate dataset for multi-view object detection. 
"""
import os 
import argparse
import numpy as np
from typing import List, Tuple, Dict
import json 

import rospy 
import cv2
import roslaunch
import rospkg
# add ./scritps directory to path
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# from src.env.simple_grounding_env import SimpleGroundingEnv
from src.env.gazebo_env import GazeboEnv
from src.env.utils import pose_msg_to_matrix
from src.perception.utils import CameraIntrinsic, Transform, project_2d_to_3d, get_instance_bbox_2d
from src.configs.config import load_config
from src.env.true_grounding_env import TrueGroundingEnv


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

def save_sensor_data(sensor_data: Dict, world_name: str, output_dir: str):
    """save all sensor data to dataset
    """
    if not os.path.exists(os.path.join(output_dir, 'rgb_images')):
        os.makedirs(os.path.join(output_dir, 'rgb_images'))
    if not os.path.exists(os.path.join(output_dir, 'depth_images')):
        os.makedirs(os.path.join(output_dir, 'depth_images'))
        
    rgb_images = []
    for i, camera in enumerate(sensor_data['camera_names']):
        rgb = sensor_data['rgb_image_list'][i]
        depth = sensor_data['depth_image_list'][i]
        # save rgb image and depth image to file 
        rgb_file = os.path.join(output_dir, 'rgb_images', world_name + '_' + camera + '.png')
        depth_file = os.path.join(output_dir, 'depth_images', world_name + '_' + camera + '.png')     
        cv2.imwrite(rgb_file, rgb[..., ::-1]) 
        # convert 32FC1 to 16UC1 and save as png
        depth = (depth * 1000).astype(np.uint16)
        cv2.imwrite(depth_file, depth)
        rgb_images.append(rgb)
           
    return rgb_images

def save_annotation(sensor_data: Dict, bbox_3d_list: List, object_names: List[str], world_name: str, output_dir: str,
                    min_bbox_size=5):
    annot_dict = {}
    if not os.path.exists(os.path.join(output_dir, 'annotations')):
        os.makedirs(os.path.join(output_dir, 'annotations'))
    
    # collect detection annotations for each camera
    for i, camera in enumerate(sensor_data['camera_names']):
    
        depth = sensor_data['depth_image_list'][i]
        intrinsics: CameraIntrinsic = sensor_data['depth_camera_intrinsic_list'][i]
        extrinsics: Transform = sensor_data['depth_camera_extrinsic_list'][i]
    
        camera_info_dict = intrinsics.to_dict()
        camera_info_dict.update(extrinsics.to_dict())
        annot_dict[camera] = {
            'camera_info': camera_info_dict,
            'detections': {}
        }
        
        # back-project depth image to 3D point cloud
        points, pixel_coords = project_2d_to_3d(depth, intrinsics, extrinsics)
        
        # generate 2d bbox for each object 
        for j, object_name in enumerate(object_names):

            bbox_3d_center, bbox_3d_size = bbox_3d_list[j]
            
            if bbox_3d_center is None or bbox_3d_size is None:
                annot_dict[camera]['detections'][object_name] = [] 
                continue
            
            # get the instance 2d bbox in the depth image 
            bbox_2d, instance_pixels = get_instance_bbox_2d(np.array(bbox_3d_center), np.array(bbox_3d_size), points, pixel_coords)
            
            # filter empty bbox 
            if bbox_2d is None or instance_pixels is None:
                annot_dict[camera]['detections'][object_name] = [] 
                continue
            
            # filter out too small bbox 
            if bbox_2d[2] - bbox_2d[0] < min_bbox_size or bbox_2d[3] - bbox_2d[1] < min_bbox_size:
                annot_dict[camera]['detections'][object_name] = [] 
                continue
            
            # FIXME: gazebo depth and rgb image has several pixel offset, fix it in the future in gazebo and remove the hack below
            # Currently, we just shift the bbox by 3 pixels to the left 
            if bbox_2d is not None:
                bbox_2d[0] = max(0, bbox_2d[0] - 5)
                bbox_2d[2] = max(0, bbox_2d[2] - 5)
            
            # save the 2d bbox to annotation dict 
            annot_dict[camera]['detections'][object_name] = bbox_2d.tolist()
            
    # save annotation to file
    annot_file = os.path.join(output_dir, 'annotations', world_name + '.json')
    with open(annot_file, 'w') as f:
        json.dump(annot_dict, f, indent=4) 
            
    return annot_dict
            
def save_image_with_bbox(rgb_images: np.ndarray, annot_dict: Dict, world_name: str, output_dir: str):
    
    if not os.path.exists(os.path.join(output_dir, 'annotated_images')):
        os.makedirs(os.path.join(output_dir, 'annotated_images'))
    
    for i, camera in enumerate(annot_dict.keys()):
        # copy makes contiguous memory layout
        rgb = rgb_images[i].copy()
        annot = annot_dict[camera]
        for object_name, bbox in annot['detections'].items():
            # bbox: [min_x, min_y, max_x, max_y]
            if len(bbox) > 0:
                # put green bounding box and text onto the image 
                cv2.rectangle(rgb, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
                cv2.putText(rgb, object_name, (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
        cv2.imwrite(os.path.join(output_dir, 'annotated_images', world_name + '_' + camera + '.png'), rgb[..., ::-1])

def save_dataset(env: GazeboEnv, object_names: List[str], world_name: str, output_dir: str):
    """Save raw sensor data and detection annotations to dataset
    sensor_data = {
        'camera_names': [],
        'rgb_image_list': [],
        'rgb_camera_intrinsic_list': [],
        'rgb_camera_frame_list': [],
        'rgb_camera_extrinsic_list': [],
        'depth_image_list': [],
        'depth_camera_intrinsic_list': [],
        'depth_camera_frame_list': [],
        'depth_camera_extrinsic_list': []
    }
    """
    # get sensor data from gazebo camera interface 
    sensor_data = env.get_sensor_data()
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    print("Saving data to ", output_dir)

    # get 3D bounding boxes from gazebo environment
    bbox_3d_list = []
    for object_name in object_names:
        bbox_center, bbox_size = env.get_gt_bbox(object_name)   
        bbox_3d_list.append((bbox_center, bbox_size))

    rgb_images = save_sensor_data(sensor_data, world_name, output_dir)
    
    annot_dict = save_annotation(sensor_data, bbox_3d_list, object_names, world_name, output_dir)
    
    save_image_with_bbox(rgb_images, annot_dict, world_name, output_dir)
    
def parse_args():
    """
    Parse the arguments of the program.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--world_name", type=str, default="table_cabinet_0")
    parser.add_argument("--config_file", type=str, default="perception_few_shot_gpt_3.5.yaml")
    parser.add_argument(
        "--include_filters",
        type=str,
        nargs="*",
        default=[],
        help="List of object classes to include in the dataset.",
    )
    parser.add_argument(
        "--exclude_filters",
        type=str,
        nargs="*",
        default=["table", "cabinent"],
        help="List of object classes to exclude from the dataset.",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default="./data/multiview_detection",
        help="Output directory for saving the dataset.",
    )
    parser.add_argument('--config_to_eval', default='text_gpt_4')
    parser.add_argument('--x', type=float, default=-0.10, help='x coordinate of the object')
    parser.add_argument('--y', type=float, default=-0.25, help='y coordinate of the object')
    parser.add_argument('--pull_distance', type=float, default=0.25, help='distance to pull the drawer')
    parser.add_argument('--table_height', type=float, default=0.815, help='z coordinate of the table top')
    parser.add_argument(
        "--object_name_filter",
        nargs="*",
        ########### google scan objects ############
        # default=[
        #     "panda_toy_figure",
        #     "Nintendo_Mario_toy_figure",
        #     "wood_block",
        #     "grey_medication_bottle",
        #     "white_and_brown_box",
        #     "brown_medication_bottle",
        #     "blue_medication_bottle",
        #     "brown_ceramic_cup",
        #     "toy_bus",
        #     "dog_figure",
        # ],
        ################# ycb objects ###############
        default=[
            "plum",
            "tomato_soup_can",
            "mug",
            "racquetball",
            "foam_brick",
            "orange",
            "baseball",
            "tennis_ball",
            "lemon",
            "banana"
        ],
        help="list of object names to evaluate",
    )
    
    args, unknown_args = parser.parse_known_args()

    print(
        "Generating multiview detection dataset... Please make sure that the script is executed from the instruct_to_policy pacakge root folder."
    )

    # Check for unknown arguments
    if unknown_args:
        print(f"Warning: Unknown arguments: {unknown_args}")

    return args


if __name__ == "__main__":
    args = parse_args()
    rospy.init_node("multiview_gen", log_level=rospy.DEBUG)
    
    # get models from metadata 
    metadata = get_models()

    # Initialize Envrionment and connect to gazebo 
    pkg_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    config_file = os.path.join(pkg_root, f'scripts/src/configs/{args.config_to_eval}.yaml')
    cfg_tabletop = load_config(config_file)
    
    env = TrueGroundingEnv(cfg_tabletop)
    
    # initialize the environment and interface to gazebo
    config = load_config(args.config_file)
    # disable model loading 
    config['grasp_detection']['method'] = 'heuristic' 

    env = GazeboEnv(config)
    
    object_names = env.get_gazebo_model_names()
    
    if len(args.include_filters) > 0:
        object_names.extend(args.include_filters)
    
    if len(args.exclude_filters) > 0:
        object_names = [obj_name for obj_name in object_names if obj_name not in args.exclude_filters]
    
    for model in mode
    
    # save all perception data to dataset 
    save_dataset(env, object_names, args.world_name, args.output_dir)
    
    # wait for IO to finish 
    rospy.sleep(3)
    