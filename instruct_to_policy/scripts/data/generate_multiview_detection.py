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
data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'data')

from src.env.simple_grounding_env import SimpleGroundingEnv
from src.env.utils import pose_msg_to_matrix, get_axis_aligned_bbox
from src.grasp_detection.utils import CameraIntrinsic, Transform
from src.config import cfg_tabletop

def get_2D_bbox_from_3D(bbox_center:np.ndarray, bbox_size:np.ndarray,
                          intrinsic:CameraIntrinsic, extrinsic:Transform)->np.ndarray:
    corners = np.array([
        [-1, -1, -1],
        [-1, -1,  1],
        [-1,  1, -1],
        [-1,  1,  1],
        [ 1, -1, -1],
        [ 1, -1,  1],
        [ 1,  1, -1],
        [ 1,  1,  1]
    ]) * bbox_size / 2 + bbox_center

    # project 3D bounding box onto the depth image
    corners = extrinsic.transform_point(corners)
    K = intrinsic.K 
    corners = K.dot(corners.T).T
    corners = corners[:, :2] / corners[:, 2:]
    corner_pixels = corners.astype(np.int32)
    
    # calculate the 2D bounding box of the 8 corner_pixels
    min_x = np.min(corner_pixels[:, 0])
    max_x = np.max(corner_pixels[:, 0])
    min_y = np.min(corner_pixels[:, 1])
    max_y = np.max(corner_pixels[:, 1])

    return np.array([min_x, min_y, max_x, max_y])
    
def check_occlusion(bbox_center:np.ndarray, bbox_size:np.ndarray, 
                    intrinsic:CameraIntrinsic, extrinsic:Transform,
                    depth_image:np.ndarray):
    """
    Check whether a model is occluded by other objects in the scene.
    Calculate the bbox_3d corners and project them to the depth image.
    At least 4 corners should have less depth than the corresponding pixel in the depth image.
    """
    # project 3D bounding box corners onto the depth image and get the corresponding depth values
    corners = np.array([
        [-1, -1, -1],
        [-1, -1,  1],
        [-1,  1, -1],
        [-1,  1,  1],
        [ 1, -1, -1],
        [ 1, -1,  1],
        [ 1,  1, -1],
        [ 1,  1,  1]
    ]) * bbox_size / 2 + bbox_center
    
    # project 3D bounding box onto the depth image
    corners_local = extrinsic.transform_point(corners)
    K = intrinsic.K 
    corners_pixels_homo = K.dot(corners_local.T).T
    corner_pixels = (corners_pixels_homo[:, :2] / corners_pixels_homo[:, 2:]).astype(np.int32)
    valid_pixels_mask = np.logical_and(
        np.logical_and(corner_pixels[:, 0] >= 0, corner_pixels[:, 0] < depth_image.shape[1]),
        np.logical_and(corner_pixels[:, 1] >= 0, corner_pixels[:, 1] < depth_image.shape[0])
    )
    valid_depth_pixels = corner_pixels[valid_pixels_mask]
    
    # calculate corner points depth values
    corner_depths = corners_local[:, 2]
    corner_depths = corner_depths[valid_pixels_mask]
    
    # get the masks of corners of which depth is less than the corresponding pixel in the depth image
    corner_depth_pixels = depth_image[valid_depth_pixels[:, 1], valid_depth_pixels[:, 0]]
    visible_corner_mask = corner_depths < corner_depth_pixels
    
    # if at least 4 corners are visible, return False
    return np.sum(visible_corner_mask) < 4
    
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

def save_annotation(sensor_data: Dict, bbox_3d_list: List, object_names: List[str], world_name: str, output_dir: str):
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
        
        for j, object_name in enumerate(object_names):

            bbox_center, bbox_size = bbox_3d_list[j]
            if check_occlusion(bbox_center, bbox_size, intrinsics, extrinsics, depth):
                annot_dict[camera]['detections'][object_name] = [] # empty list means no detection found
            else:
                # need to call tolist() to convert np.int32 to python int for json serialization
                annot_dict[camera]['detections'][object_name] = get_2D_bbox_from_3D(bbox_center, bbox_size, intrinsics, extrinsics).tolist()
    
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

def save_dataset(env: SimpleGroundingEnv, object_names: List[str], world_name: str, output_dir: str):
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
        object_pose = env.get_gt_obj_pose(object_name)
        object_transform = pose_msg_to_matrix(object_pose)
        bbox_center, bbox_size = get_axis_aligned_bbox(env.object_info[object_name]['bbox_center'], 
                                                    env.object_info[object_name]['bbox_size'], object_transform)
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
    parser.add_argument("--world_name", type=str, default="table_cabinet_1")
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
        default=[],
        help="List of object classes to exclude from the dataset.",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default=os.path.join(data_dir, "multiview_detection"),
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
    rospy.init_node("multiview_gen", log_level=rospy.WARN)
    
    # launch gazebo world
    # NOTE: roslaunch python API somehow does not work for urdf_spawn 
    # launch = launch_gazebo_world(args.world_name) 
    
    # wait for gazebo environment to be ready and static
    # wait for 
    rospy.sleep(3) 
    
    # initialize the environment and interface to gazebo
    config = cfg_tabletop.copy()
    # disable model loading 
    config['grasp_detection']['method'] = 'heuristic' 
    env = SimpleGroundingEnv(cfg_tabletop)
    
    # load world metadata to get objects to collect detection 
    object_names = [] 
    metadata_file = os.path.join(data_dir, 'world_metadata', args.world_name + '.json')
    with open(metadata_file, 'r') as f:
        metadata = json.load(f)
        for object_id, object_info in metadata.items():
            # TODO: consider if we need data for drawer detection 
            if object_info['type'] not in ["env", "robot", "cabinet", "drawer"]:
                object_names.append(object_info['name'])
    
    if len(args.include_filters) > 0:
        object_names.extend(args.include_filters)
    
    if len(args.exclude_filters) > 0:
        object_names = [obj_name for obj_name in object_names if obj_name not in args.exclude_filters]
    
    # save all perception data to dataset 
    save_dataset(env, object_names, args.world_name, args.output_dir)
    
    # wait for IO to finish 
    rospy.sleep(3)
    