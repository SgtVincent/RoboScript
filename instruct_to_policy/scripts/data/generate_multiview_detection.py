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
    
def launch_gazebo_world(world_name: str):
    """
    Start the gazebo world with the given world name with roslauch.
    """
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('instruct_to_policy')
    
    cli_args = [
        'instruct_to_policy', 'run_panda_moveit_gazebo.launch',
        f'world:={package_path}/worlds/{world_name}.world',
        'enable_moveit:=false',
    ]
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    roslaunch_args = cli_args[2:]

    launch = roslaunch.parent.ROSLaunchParent(uuid, [(roslaunch_file, roslaunch_args)])

    launch.start()
    rospy.loginfo(f"Gazebo world {world_name} started")

    return launch

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
    
    # wait for gazebo environment to be ready and static
    rospy.sleep(3) 
    
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
    
    # save all perception data to dataset 
    save_dataset(env, object_names, args.world_name, args.output_dir)
    
    # wait for IO to finish 
    rospy.sleep(3)
    