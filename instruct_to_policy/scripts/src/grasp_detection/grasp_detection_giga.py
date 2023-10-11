import os
from typing import List, Tuple, Dict
from typing import Dict 
import numpy as np 
import trimesh 

# Make sure to install giga repo in conda environment 
from vgn.detection_implicit import VGNImplicit
from vgn.experiments.clutter_removal import State
from vgn.perception import TSDFVolume, ScalableTSDFVolume
from vgn.utils.visual import grasp2mesh, plot_voxel_as_cloud, plot_tsdf_with_grasps
from vgn.utils.implicit import get_mesh_pose_list_from_world, get_scene_from_mesh_pose_list

from .utils import CameraIntrinsic, Transform, Rotation, get_mask_from_2D_bbox, open3d_frustum_filter


from .grasp_detection_base import GraspDetectionBase

class GraspDetectionGIGA(GraspDetectionBase):
    """
    Wrapper class for GIGA grasp detection.
    """
    def __init__(self, config, **kwargs):
        super(GraspDetectionGIGA, self).__init__(config)
        self.model_path = self.config["model_path"]
        self.model_type = self.config["model_type"]
        
        # uniform volume cannot handle large scene reconstruction
        self.volume_type = self.config.get("volume_type", "scalable")
        self.voxel_grid_size = self.config.get("voxel_grid_size", 0.3) # in meters
        self.resolution = self.config.get("resolution", 40)
        
        self.qual_th = self.config.get("quality_threshold", 0.8)
        self.out_th = self.config.get("outlier_voxel_threshold", 0.1)
        
        self.visualize = kwargs.get("visualize", False)
        self.verbose = kwargs.get("verbose", False)
        
        
    def load_model(self):
        
        self.model = VGNImplicit(self.model_path,
                    self.model_type,
                    best=True, # rank grasps by score
                    qual_th=self.qual_th,
                    visualize=False, # DO NOT use vgn visualization since gt mesh not available in the pipeline 
                    force_detection=True,
                    out_th=self.out_th,
                    resolution=self.resolution)
      
      
    def _preprocess(self, data: Dict):
        """
        Preprocess data for grasp detection:
        - integrate depth images into TSDF volume
        - return TSDF volume and point cloud
        """
        if self.volume_type == "uniform":
            tsdf = TSDFVolume(self.voxel_grid_size, self.resolution)
        elif self.volume_type == "scalable":
            tsdf = ScalableTSDFVolume(self.voxel_grid_size, self.resolution)

        for i, camera in enumerate(data['camera_names']):

            rgb = None
            depth = data['depth_image_list'][i]
            intrinsics: CameraIntrinsic = data['depth_camera_intrinsic_list'][i]
            extrinsics: Transform = data['depth_camera_extrinsic_list'][i]
        
            mask=None
            if "depth_bboxes" in data:
                bbox = data['depth_bboxes'][i]
                mask = get_mask_from_2D_bbox(bbox, depth) 

            tsdf.integrate(depth, intrinsics, extrinsics, rgb_img=rgb, mask_img=mask)

        if self.volume_type == "scalable":
            
            if 'bbox_3d' in data:
                bbox_3d_center = data['bbox_3d']['center']
                bbox_3d_size = data['bbox_3d']['size']
            else:
                # filter the cloud with frustum filters and get the bounding box center 
                assert 'bbox_2d_list' in data, 'bbox_2d_list must be provided if bbox_3d is not provided'
                full_cloud = tsdf._volume.extract_point_cloud()
                filtered_pc, mask = open3d_frustum_filter(full_cloud, data['bbox_2d_list'],
                                                            data['depth_camera_intrinsic_list'],
                                                            data['depth_camera_extrinsic_list'])
                bbox_3d_center = filtered_pc.get_center()
                bbox_3d_size = self.voxel_grid_size * np.ones((3))
            
            tsdf.set_region_of_interest(bbox_3d_center, bbox_3d_size)
            
        pc = tsdf.get_cloud()
        
        return tsdf, pc 
    
        
    def predict(self, data: Dict)-> Tuple[List, List, List]:
        
        tsdf, pc = self._preprocess(data)
        state = State(tsdf, pc)
        
        # if self.visualize: 
            # mesh_pose_list = get_mesh_pose_list_from_world(sim.world, object_set)
            # scene_mesh = get_scene_from_mesh_pose_list(mesh_pose_list)
            # grasps, scores, _, composed_scene = self.model(state, scene_mesh)
            # visualize composed scene
            # composed_scene.show(viewer='gl') 
            
        # else:
        grasps, scores, _ = self.model(state)
        
        print(f"{len(grasps)} Grasps generated.")

        # visualize grasps
        if self.visualize:
            if len(grasps) > 0:
                fig = plot_tsdf_with_grasps(tsdf.get_grid()[0], [grasps[0]], lim=[self.voxel_grid_size] * 3)
                print(scores)
            else:
                fig = plot_voxel_as_cloud(tsdf.get_grid()[0], lim=[self.voxel_grid_size] * 3)
            fig.show()

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
        
    
        
        

