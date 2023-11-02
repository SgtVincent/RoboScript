"""
This script is used to demo grasp detection pipeline:
- Given a 3D bounding box in global frame as input.
- Generate TSDF from RGB-Depth images of gazebo camera sets.  
- Predict grasp pose from TSDF with GIGA model.

TODO: incorporate this script as a Perception class and embed it into the GazeboEnv class
"""

import os 
from typing import List, Tuple, Dict
import argparse
import numpy as np
import open3d as o3d
import genpy
from pathlib import Path
import pickle
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from collections import deque    

# ROS related
import rospy

from vgn.detection_implicit import VGNImplicit
from vgn.experiments.clutter_removal import State
from vgn.perception import TSDFVolume, ScalableTSDFVolume
from vgn.utils.visual import grasp2mesh, plot_voxel_as_cloud, plot_tsdf_with_grasps

from src.env.gazebo_cameras import GazeboRGBDCameraSet, GazeboRGBDCamera
from src.perception.utils import Transform, Rotation, CameraIntrinsic, get_mask_from_3D_bbox
  
def predict_grasp(args, planner, data: Dict):
    # DO NOT use color since realsense D435 camera has different color and depth image resolution & optical center
    if args.volume_type == "uniform":
        tsdf = TSDFVolume(args.size, args.resolution)
    elif args.volume_type == "scalable":
        tsdf = ScalableTSDFVolume(args.size, args.resolution)

    for i, camera in enumerate(data['camera_names']):

        if args.color_type == "rgb":
            rgb = data['rgb_image_list'][i]
        else:
            rgb = None
        depth = data['depth_image_list'][i]
        intrinsics: CameraIntrinsic = data['depth_camera_intrinsic_list'][i]
        extrinsics: Transform = data['depth_camera_extrinsic_list'][i]
    
        # intrinsics = CameraIntrinsic.from_dict(intrinsics)
        # extrinsics = Transform.from_matrix(extrinsics)

        # calculate mask image from 3D bounding box 
        mask = None
        if args.use_depth_mask:
            mask = get_mask_from_3D_bbox(np.array(args.object_center), np.array(args.object_size), depth, intrinsics, extrinsics)

        tsdf.integrate(depth, intrinsics, extrinsics, rgb_img=rgb, mask_img=mask)

    if args.volume_type == "scalable":
        tsdf.set_region_of_interest(np.array(args.object_center), np.array(args.object_size))

    pc = tsdf.get_cloud()
    state = State(tsdf, pc)
    grasps, scores, _ = planner(state)
    print(len(grasps))

    # visualize grasps
    # plt.ion()
    if len(grasps) > 0:
        fig = plot_tsdf_with_grasps(tsdf.get_grid()[0], [grasps[0]])
        print(scores)
    else:
        fig = plot_voxel_as_cloud(tsdf.get_grid()[0])
    fig.show()
    # while True:
    #     if plt.waitforbuttonpress():
    #         break
    # plt.close(fig)


    # grasp_meshes = [
    #     grasp2mesh(grasps[idx], 1).as_open3d for idx in range(len(grasps))
    # ]
    # geometries = [pc] + grasp_meshes

    # from copy import deepcopy
    # grasp_bck = deepcopy(grasps[0])
    # grasp_mesh_bck = grasp2mesh(grasp_bck, 1).as_open3d
    # grasp_mesh_bck.paint_uniform_color([0, 0.8, 0])

    # pos = grasps[0].pose.translation
    # # pos[2] += 0.05
    # angle = grasps[0].pose.rotation.as_euler('xyz')
    # print(pos, angle)
    # if angle[2] > np.pi / 2 or angle[2] < - np.pi / 2:
    #     reflect = Transform(Rotation.from_euler('xyz', (0, 0, np.pi)), np.zeros((3)))
    #     grasps[0].pose = grasps[0].pose * reflect
    # pos = grasps[0].pose.translation
    # angle = grasps[0].pose.rotation.as_euler('xyz')
    # print(pos, angle)
    # # grasps[0].pose = Transform(Rotation.from_euler('xyz', (angle[0], angle[1], angle[2])), pos)
    # grasp_mesh = grasp2mesh(grasps[0], 1).as_open3d
    # grasp_mesh.paint_uniform_color([0.8, 0, 0])
    # geometries = [tsdf.get_mesh(), grasp_mesh, grasp_mesh_bck]

    # shift grasp translation by tsdf origin
    tsdf_origin = tsdf.cropped_grid.origin
    for i, grasp in enumerate(grasps):
        grasps[i].pose.translation = grasp.pose.translation + tsdf_origin

    if len(grasps) == 0:
        # return [], [], [tsdf.get_mesh()]
        return [], [], [tsdf.get_cloud()]
    pos = grasps[0].pose.translation
    # pos[2] += 0.05
    angle = grasps[0].pose.rotation.as_euler('xyz')
    print(pos, angle)
    if angle[2] > np.pi / 2 or angle[2] < - np.pi / 2:
        reflect = Transform(Rotation.from_euler('xyz', (0, 0, np.pi)), np.zeros((3)))
        grasps[0].pose = grasps[0].pose * reflect
    pos = grasps[0].pose.translation
    angle = grasps[0].pose.rotation.as_euler('xyz')
    print(pos, angle)
    # grasps[0].pose = Transform(Rotation.from_euler('xyz', (angle[0], angle[1], angle[2])), pos)
    grasp_mesh = grasp2mesh(grasps[0], 1).as_open3d
    grasp_mesh.paint_uniform_color([0, 0.8, 0])
    # geometries = [tsdf.get_mesh(), grasp_mesh]
    geometries = [tsdf.get_cloud(), grasp_mesh]
    #exit(0)
    return grasps, scores, geometries
    
def visualize_point_clouds(args, data: Dict, use_mask=False):
    """
    This function is used to visualize all point clouds from all depth images 
    """
    gt_pcl_list = []
    pcl_list = []
    for i, camera in enumerate(data['camera_names']):
        depth = data['depth_image_list'][i]
        intrinsic: CameraIntrinsic = data['depth_camera_intrinsic_list'][i]
        extrinsic: Transform = data['depth_camera_extrinsic_list'][i]
        gt_points = data['points'][i]
        # reconstruct point cloud from depth image
        
        if use_mask:
            mask = get_mask_from_3D_bbox(np.array(args.object_center), np.array(args.object_size), depth, intrinsic, extrinsic)
            depth = depth * mask
        
        o3d_pcl = o3d.geometry.PointCloud.create_from_depth_image(
            depth = o3d.geometry.Image(depth),
            intrinsic= o3d.camera.PinholeCameraIntrinsic(
                width=intrinsic.width,
                height=intrinsic.height,
                fx=intrinsic.fx,
                fy=intrinsic.fy,
                cx=intrinsic.cx,
                cy=intrinsic.cy,
            ),
            extrinsic=extrinsic.as_matrix(),
            depth_scale=1.0,
            depth_trunc=3.0,
        )
        pcl_list.append(o3d_pcl)
        
        # create GT point cloud from gt_points
        gt_pcl = o3d.geometry.PointCloud()
        gt_pcl.points = o3d.utility.Vector3dVector(gt_points)
        gt_pcl_list.append(gt_pcl)

        # visualize two point clouds
    o3d.visualization.draw_geometries(pcl_list)

def compute_projection_error(point_cloud, depth_image, intrinsic, extrinsic, visualize=False):
    """
    This function is used to compute reprojection error of a point cloud
    """
    # Project 3D points to 2D image coordinates
    homogeneous_coords = np.hstack((point_cloud, np.ones((point_cloud.shape[0], 1))))
    projected_points = np.dot(extrinsic, homogeneous_coords.T)
    projected_points = projected_points[:3, :] / projected_points[3, :]
    projected_points = np.dot(intrinsic, projected_points)
    projected_points = projected_points[:2, :]

    # Ensure the projected points are within the image bounds
    height, width = depth_image.shape
    projected_points[0, :] = np.clip(projected_points[0, :], 0, width - 1)
    projected_points[1, :] = np.clip(projected_points[1, :], 0, height - 1)

    # Compute the projected depth image
    projected_depth = depth_image[projected_points[1, :].astype(np.int), projected_points[0, :].astype(np.int)]

    if visualize:
        # visualize depth images on the fly for debugging
        plt.subplot(1, 2, 1)
        plt.imshow(depth_image)
        plt.subplot(1, 2, 2)
        plt.imshow(projected_depth.reshape(depth_image.shape))
        plt.show()

    error = projected_depth - point_cloud[:, 2]
    # Compute the mean and root mean square error
    mean_error = np.mean(error)
    rmse = np.sqrt(np.mean(error ** 2))

    print("Mean Projection Error:", mean_error)
    print("Root Mean Square Error:", rmse)

    return mean_error


def main(args):

    # initialize ros node
    rospy.init_node('grasp_from_gazebo_sensors', anonymous=False)

    # initialize grasp perdiction model
    planner = VGNImplicit(args.model,
                        args.type,
                        best=True,
                        qual_th=0.8,
                        rviz=False,
                        force_detection=True,
                        out_th=0.1,
                        resolution=args.resolution)

    # initialize gazebo camera set
    # camera_set = GazeboRGBDCameraSet(cameras_list=args.camera_names, namespace=args.namespace)
    camera_set = GazeboRGBDCameraSet(cameras_list=args.camera_names, namespace=args.namespace, sub_pcl=True)

    while True:
        data = camera_set.get_latest_data()

        # visualize_point_clouds(args, data, use_mask=True)

        # compute reprojection error
        # for i, camera in enumerate(data['camera_names']):
        #     depth = data['depth_image_list'][i]
        #     intrinsic: CameraIntrinsic = data['depth_camera_intrinsic_list'][i]
        #     extrinsic: Transform = data['depth_camera_extrinsic_list'][i]
        #     gt_points = data['points'][i]
        #     error = compute_projection_error(gt_points, depth, intrinsic.K, extrinsic.as_matrix(), visualize=True)

        grasps, scores, geometries = predict_grasp(args, planner, data)

        #o3d.visualization.draw_geometries(geometries, zoom=1.0, front=[0, -1, 0], up=[0, 0, 1], lookat=[0.15, 0.15, 0.05], mesh_show_back_face=True)
        # o3d.visualization.draw_geometries(geometries)
        o3d.visualization.draw_geometries(geometries)
        # output_path = input_path.replace('.pkl', 'grasps.pkl')
        # with open(output_path, 'wb') as f:
        #     pickle.dump({'grasps': grasps, 'scores': scores}, f)
        # send_msg(['output', output_path])
        pass


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    # parser.add_argument("--model", type=Path, default="data/models/pile_convonet_v_cat_occ.pt")
    parser.add_argument("--model", type=Path, default="data/models/giga_pile.pt")
    parser.add_argument("--type", type=str, default="giga")
    parser.add_argument("--color_type", type=str, default="depth", choices=["rgb", "depth"])
    parser.add_argument("--volume_type", type=str, default="scalable", choices=["uniform", "scalable"]) # do not use scalable for now
    parser.add_argument("--use_depth_mask", action="store_true")
    parser.add_argument("--size", type=float, default=0.3)
    parser.add_argument("--resolution", type=float, default=40)
    parser.add_argument("--lower", type=float, nargs=3, default=[0.02, 0.02, 0.005])
    parser.add_argument("--upper", type=float, nargs=3, default=[0.28, 0.28, 0.3])
    parser.add_argument("--object_center", type=float, nargs=3, default=[0.244, -0.383, 1.014])
    parser.add_argument("--object_size", type=float, nargs=3, default=[0.3, 0.3, 0.3]) # best choice: size, 0.3
    # parser.add_argument("--object_center", type=float, nargs=3, default=[-0.177, -0.44, 1.071])
    # parser.add_argument("--object_size", type=float, nargs=3, default=[0.3, 0.3, 0.3]) 
    parser.add_argument("--camera_names", type=str, nargs='+', default=['camera_left', 'camera_right', 'camera_top'])
    parser.add_argument("--namespace", type=str, default="")
    args = parser.parse_args()

    args.use_depth_mask = False

    main(args)
