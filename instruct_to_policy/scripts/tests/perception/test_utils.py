import os
import numpy as np
import open3d as o3d
import cv2
import pytest

# add catkin_ws/devel/lib into sys.path
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../../../../devel/lib/python3.9/site-packages'))

from src.perception.scalable_tsdf import ScalableTSDFVolume
from src.perception.utils import (
    CameraIntrinsic,
    Transform,
    load_data_from_multiview_detection,
    match_bboxes_clustering,
    match_bboxes_points_matching,
    draw_multiview_bbox_matches
)

DEBUG_VISUALIZE = True

@pytest.fixture
def test_data():
    # Define the test parameters
    # world_name = "world_1_table_sort"
    # world_name = "world_3_mug_to_empty_plate"
    world_name = "world_6_mug_to_same_color_plate"
    exclude_keywords = ["cabinet"]
    current_dir = os.path.dirname(os.path.realpath(__file__))
    data_dir = os.path.join(current_dir, f"../../../data/benchmark/multiview_detection")

    data = load_data_from_multiview_detection(
        data_dir=data_dir,
        world_name=world_name,
        exclude_keywords=exclude_keywords
    )

    # Fuse the perception data to tsdf
    scene_tsdf_masked = ScalableTSDFVolume(
        voxel_size=0.005,
        color_type="rgb",
        trunc_ratio=4.0
    )

    for i, name in enumerate(data['camera_names']):

        # convert sensor_msgs/Image
        rgb = data['rgb_image_list'][i]
        depth = data['depth_image_list'][i]
        detections = data['detections_list'][i]
        bbox_2d_list = list(detections.values())

        # NOTE: assume the depth has been aligned with rgb
        assert rgb.shape[0] == depth.shape[0] and rgb.shape[1] == depth.shape[1]
        intrinsic = data['depth_camera_intrinsic_list'][i]
        extrinsic = data['depth_camera_extrinsic_list'][i]

        # calculate the mask image with all bounding boxes inner regions set to 1
        mask_img = np.zeros_like(depth)
        for bbox in bbox_2d_list:
            mask_img[bbox[1]:bbox[3], bbox[0]:bbox[2]] = 1

        scene_tsdf_masked.integrate(depth, intrinsic, extrinsic, rgb_img=rgb, mask_img=mask_img)

    return {
        'data': data,
        'scene_tsdf_masked': scene_tsdf_masked,
        'world_name': world_name
    }

def calculate_result_precision(result, gt_match_tuple_list):
    # the order of tuples in result and gt_match_tuple_list may be different
    # but the order inside each tuple should be the same

    # calculate the precision of the result
    num_correct_match = 0
    for match_tuple in result:
        if match_tuple in gt_match_tuple_list:
            num_correct_match += 1
    precision = num_correct_match / len(result)
    return precision

def calculate_result_recall(result, gt_match_tuple_list):
    # the order of tuples in result and gt_match_tuple_list may be different
    # but the order inside each tuple should be the same

    # calculate the recall of the result
    num_correct_match = 0
    for match_tuple in result:
        if match_tuple in gt_match_tuple_list:
            num_correct_match += 1
    recall = num_correct_match / len(gt_match_tuple_list)
    return recall

def get_gt_match_tuple_list(bboxes_labels_list):
    # bboxes_labels_list contains the bounding box labels for each camera view
    # get ground truth match by putting all bounding boxes of the same label into a tuple, and put all tuples into a list
    gt_match_tuple_list = []
    unique_labels = set()
    for bboxes_labels in bboxes_labels_list:
        unique_labels.update(bboxes_labels)

    for label in unique_labels:
        bbox_tuple = []
        for i, bboxes_labels in enumerate(bboxes_labels_list):
            if label in bboxes_labels:
                # the tuple should contain the bounding box index in bboxes_labels_list
                bbox_tuple.append(bboxes_labels.index(label))
            else:
                # if the label does not exist in the current camera view, set the index to -1
                bbox_tuple.append(-1)
        gt_match_tuple_list.append(tuple(bbox_tuple))
    
    return gt_match_tuple_list


def test_match_bboxes_clustering(test_data):
    data = test_data['data']
    scene_tsdf_masked = test_data['scene_tsdf_masked']
    world_name = test_data['world_name']

    # prepare function input and expected output
    bboxes_2d_list = [
        list(detections.values()) for detections in data['detections_list']
    ]
    bboxes_labels_list = [
        list(detections.keys()) for detections in data['detections_list']
    ]

    gt_match_tuple_list = get_gt_match_tuple_list(bboxes_labels_list)

    # Call the function with the inputs
    result = match_bboxes_clustering(
        bboxes_2d_list=bboxes_2d_list,
        intrinsics=data['depth_camera_intrinsic_list'],
        extrinsics=data['depth_camera_extrinsic_list'],
        pcl=scene_tsdf_masked.get_cloud(),
        downsample=False,
        min_samples=10,
        cluster_eps=0.02,
        debug_visualize=DEBUG_VISUALIZE
    )

    # calculate the precision of the result
    precision = calculate_result_precision(result, gt_match_tuple_list)
    recall = calculate_result_recall(result, gt_match_tuple_list)
    print(f"Function match_bboxes_clustering on world {world_name} get precision( {precision} ), recall( {recall} )")

    # NOTE: when there are multiple objects on the table, the peripheral points of the objects may be clustered together
    # making this method fail to distinguish the objects
    assert np.isclose(precision, 1.0, atol=0.1)

def test_match_bboxes_points_matching(test_data):
    data = test_data['data']
    scene_tsdf_masked = test_data['scene_tsdf_masked']
    world_name = test_data['world_name']

    # prepare function input and expected output
    bboxes_2d_list = [
        list(detections.values()) for detections in data['detections_list']
    ]
    bboxes_labels_list = [
        list(detections.keys()) for detections in data['detections_list']
    ]

    gt_match_tuple_list = get_gt_match_tuple_list(bboxes_labels_list)

    # Call the function with the inputs
    result = match_bboxes_points_matching(
        bboxes_2d_list=bboxes_2d_list,
        intrinsics=data['depth_camera_intrinsic_list'],
        extrinsics=data['depth_camera_extrinsic_list'],
        pcl=scene_tsdf_masked.get_cloud(),
        downsample=True,
        downsample_voxel_size=0.02,
        min_match_th=0.1,
        debug_visualize=DEBUG_VISUALIZE
    )

    # calculate the precision of the result
    precision = calculate_result_precision(result, gt_match_tuple_list)
    recall = calculate_result_recall(result, gt_match_tuple_list)
    print(f"Function match_bboxes_points_matching on world {world_name} get precision( {precision} ), recall( {recall} )")

    if DEBUG_VISUALIZE:
        match_result_canvas = draw_multiview_bbox_matches(
            rgb_image_list=data['rgb_image_list'],
            bboxes_2d_list=bboxes_2d_list,
            matched_bboxes_idx_tuple_list=result
        )
        # cv2.imshow("match_result_canvas", match_result_canvas[..., ::-1])
        # convert BGR to RGB
        match_result_canvas = cv2.cvtColor(match_result_canvas, cv2.COLOR_BGR2RGB)
        # save the result image
        cv2.imwrite(f"match_bboxes_{world_name}_prec_{precision}_recall_{recall}.png", match_result_canvas)

    assert np.isclose(precision, 1.0, atol=0.1) and np.isclose(recall, 1.0, atol=0.1)
