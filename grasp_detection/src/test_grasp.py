#!/usr/bin/env python3
import rospy
import os 
import numpy as np
import open3d as o3d
import cv2
import json 
import cv_bridge

from grasp_detection.srv import DetectGrasps, DetectGraspsRequest, DetectGraspsResponse
from grasp_detection.msg import Perception, PerceptionSingleCamera

from detectors.utils import CameraIntrinsic, Transform, Rotation, get_mask_from_2D_bbox, open3d_frustum_filter, data_to_percetion_msg

def load_data_to_perception_msgs(dataset_dir, camera_names, world_name, object_name):
    """
    Load data from dataset folder and convert to Perception data type
    
    dataset_dir
    ├── annotated_images
    ├── annotations
    ├── depth_images
    └── rgb_images
    """
    # First load data into data dict
    data = {
        'camera_names': [],
        'rgb_image_list': [],
        'rgb_camera_intrinsic_list': [],
        'rgb_camera_frame_list': [],
        'rgb_camera_extrinsic_list': [],
        'depth_image_list': [],
        'depth_camera_intrinsic_list': [],
        'depth_camera_frame_list': [],
        'depth_camera_extrinsic_list': [],
        'depth_bboxes':[],
        'bbox_3d':{
            'center': [],
            'size': [],
        },
    }
    
    with open(os.path.join(dataset_dir, 'annotations', world_name + '.json'), 'r') as f:
        annotations = json.load(f)
    
    for camera_name in camera_names:
        data['camera_names'].append(camera_name)
        
        # rgb image
        rgb_image = cv2.imread(os.path.join(dataset_dir, 'rgb_images', world_name + '_' + camera_name + '.png'))
        data['rgb_image_list'].append(rgb_image)
        
        # depth image
        depth_image = cv2.imread(os.path.join(dataset_dir, 'depth_images', world_name + '_' + camera_name + '.png'), cv2.IMREAD_UNCHANGED)
        # convert 16UC1 to 32FC1 
        depth_image = depth_image.astype(np.float32) / 1000.0
        data['depth_image_list'].append(depth_image)
        
        # import camera intrinsics and extrinics from annotations
        camera_info = annotations[camera_name]['camera_info']
        width = camera_info['width']
        height = camera_info['height']
        K = camera_info['K']
        rotation = camera_info['rotation']
        translation = camera_info['translation']
        camera_intrinsic = CameraIntrinsic(
            width=width,
            height=height,
            fx=K[0],
            fy=K[4],
            cx=K[2],
            cy=K[5],   
        )
        camera_extrinsic = Transform.from_dict(
            {
                "translation": translation,
                "rotation": rotation,
            }
        )
        
        # use aligned depth image, so the depth camera frame is the same as rgb camera frame
        data['rgb_camera_intrinsic_list'].append(camera_intrinsic)
        data['depth_camera_intrinsic_list'].append(camera_intrinsic)
        data['rgb_camera_extrinsic_list'].append(camera_extrinsic)
        data['depth_camera_extrinsic_list'].append(camera_extrinsic)
        
        # always use world frame 
        data['rgb_camera_frame_list'].append('world')
        data['depth_camera_frame_list'].append('world')
        
        # object detection
        depth_bbox = annotations[camera_name]['detections'][object_name]
        data['depth_bboxes'].append(depth_bbox)
    
    bridge = cv_bridge.CvBridge()
    perception_msg = data_to_percetion_msg(data, bridge) 
    
    return perception_msg
    

if __name__ == "__main__":

    rospy.init_node('test_grasp')

    dataset_dir = rospy.get_param('~dataset_dir', '/home/junting/franka_ws/src/franka_fisher/instruct_to_policy/data/multiview_detection')
    camera_names = ['camera_left', 'camera_top', 'camera_right']
    world_name = rospy.get_param('~world_name', 'table_cabinet_4')
    object_name = rospy.get_param('~object_name', 'mustard_bottle')
    service_name = rospy.get_param('~service_name', '/detect_grasps')
    
    rospy.wait_for_service(service_name)
    detect_grasps = rospy.ServiceProxy(service_name, DetectGrasps)
    
    # load annotation to get all detected objects
    with open(os.path.join(dataset_dir, 'annotations', world_name + '.json'), 'r') as f:
        annotations = json.load(f)
        # find objects that have been detected in all cameras
        object_names = []
        for obj in annotations[camera_names[0]]['detections'].keys():
            if all([len(annotations[camera_name]['detections'][obj]) > 0 for camera_name in camera_names]):
                object_names.append(obj)

    print(f"Detected objects in all cameras: {object_names}")

    perception_msg = load_data_to_perception_msgs(dataset_dir, camera_names, world_name, object_name)
    
    print(f"Loaded perception data for {object_name}")
    
    # Call service to detect grasps
    request = DetectGraspsRequest(perception_data=perception_msg)
    response:DetectGraspsResponse = detect_grasps(request)

    # print top 5 candidates 
    predicted_grasps = response.grasps[:5]
    grasp_scores = [grasp.grasp_score for grasp in predicted_grasps]
    grasp_poses = [grasp.grasp_pose for grasp in predicted_grasps]
    sort_idx = np.argsort(grasp_scores)[::-1]
    for i in range(5):
        print(f"Grasp {i}: score={grasp_scores[sort_idx[i]]}, pose={grasp_poses[sort_idx[i]]}")
        
    # TODO: Visualize grasps
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(perception_data[:, :3])
    # pcd.colors = o3d.utility.Vector3dVector(perception_data[:, 3:6] / 255.0)

    # o3d.visualization.draw_geometries([pcd])

    # for grasp in predicted_grasps:
    #     grasp_points = np.array(grasp.points)
    #     grasp_line_set = o3d.geometry.LineSet()
    #     grasp_line_set.points = o3d.utility.Vector3dVector(grasp_points)
    #     grasp_line_set.lines = o3d.utility.Vector2iVector([[0, 1], [1, 2], [2, 3], [3, 0], [0, 2], [1, 3]])
    #     grasp_line_set.colors = o3d.utility.Vector3dVector(np.tile([1, 0, 0], (6, 1)))
    #     o3d.visualization.draw_geometries([pcd, grasp_line_set])
