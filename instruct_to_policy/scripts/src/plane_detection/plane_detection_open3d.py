import os
from re import L
from typing import Dict, List, Tuple
import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d

from .plane_detection_base import PlaneDetectionBase

class PlaneDetectionOpen3D:
    def __init__(self, model_params: Dict):
        self.model_params = model_params
        self.normal_variance_threshold_deg = model_params.get(
            "normal_variance_threshold_deg", 60
        )
        self.coplanarity_deg = model_params.get("coplanarity_deg", 75)
        self.outlier_ratio = model_params.get("outlier_ratio", 0.75)
        self.min_plane_edge_length = model_params.get("min_plane_edge_length", 0.0)
        self.min_num_points = model_params.get("min_num_points", 0)
        self.search_param_knn = model_params.get("search_param_knn", 30)

    def load_model(self):
        # This class uses a statistical approach to plane detection, so no model to load
        # Ref: A. Araújo and M. Oliveira, A robust statistics approach for plane detection in unorganized point clouds, Pattern Recognition, 2020
        pass

    def detect_planes(self, pcl_o3d: o3d.geometry.PointCloud)-> Tuple[List[np.ndarray], List[o3d.geometry.OrientedBoundingBox]]:
        """
        Detects planar patches in the point cloud using a robust statistics-based approach using open3d detect_planar_patches interface.
        Args:
            pcl_o3d: o3d.geometry.PointCloud, open3d point cloud

        open3d.geometry.PointCloud.detect_planar_patches(self, normal_variance_threshold_deg=60, 
            coplanarity_deg=75, outlier_ratio=0.75, min_plane_edge_length=0.0, min_num_points=0, 
            search_param=KDTreeSearchParamKNN with knn = 30)
            
            PARAMETERS:
                normal_variance_threshold_deg (float, optional, default=60)
                    coplanarity_deg (float, optional, default=75)
                outlier_ratio (float, optional, default=0.75)  Maximum allowable ratio of outliers associated to a plane.
                min_plane_edge_length (float, optional, default=0.0) Minimum edge length of plane's long edge before being rejected.
                min_num_points (int, optional, default=0) Minimum number of points allowable for fitting planes.
                search_param (open3d.geometry.KDTreeSearchParam, optional, default=KDTreeSearchParamKNN with knn = 30) 
                    The KDTree search parameters for neighborhood search.
            RETURNS:
                List[open3d.geometry.OrientedBoundingBox] A list of detected planar patches, represented as OrientedBoundingBox objects, 
                    with the third column (z) of R indicating the planar patch normal vector. 
                    The extent in the z direction is non-zero so that the OrientedBoundingBox contains the points that contribute to the plane detection.
        """
        oboxes = pcl_o3d.detect_planar_patches(
            normal_variance_threshold_deg=self.normal_variance_threshold_deg,
            coplanarity_deg=self.coplanarity_deg,
            outlier_ratio=self.outlier_ratio,
            min_plane_edge_length=self.min_plane_edge_length,
            min_num_points=self.min_num_points,
            search_param=o3d.geometry.KDTreeSearchParamKNN(knn=self.search_param_knn))
        
        # The third column (z) of R indicating the planar patch normal vector 
        normal_vectors = []
        for obox in oboxes:
            normal_vectors.append(obox.R[:, 2])
            
        return normal_vectors, oboxes

    def visualize_planes(self, pcd:o3d.geometry.PointCloud, oboxes: List[o3d.geometry.OrientedBoundingBox], normal_vectors: np.ndarray):
        '''
        Draw the point cloud, detected planar patches, and their normals.
        
        Args:
            pcd: o3d.geometry.PointCloud, open3d point cloud
            oboxes: List[o3d.geometry.OrientedBoundingBox], list of detected planar patches, represented as OrientedBoundingBox objects
            normal_vectors: np.ndarray, array of normal vectors for each detected planar patch

        '''
        geometries = []
        # visualize the patches, bounding boxes, and normals
        for i, obox in enumerate(oboxes):
            mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.1])
            mesh.paint_uniform_color(obox.color)
            normal_mesh = o3d.geometry.TriangleMesh.create_arrow(
                cylinder_radius=0.02,
                cone_radius=0.05,
                cylinder_height=0.3,
                cone_height=0.1
            )
            # move mesh to obox center
            normal_mesh.translate(obox.get_center())
            # rotate mesh to align with obox normal at the obox center
            o3d.geometry.get_rotation_matrix_from_axis_angle(normal_vectors[i, :])
            rotaion_matrix = R.align_vectors([normal_vectors[i]], [[0, 0, 1]],)[0].as_matrix()
            normal_mesh.rotate(rotaion_matrix, center=obox.get_center())
            # set normal arrow color to obox color
            normal_mesh.paint_uniform_color(obox.color)
            geometries.append(mesh)
            geometries.append(obox)
            geometries.append(normal_mesh)
        geometries.append(pcd)

        # also add coords frame 
        coords = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        geometries.append(coords)

        o3d.visualization.draw_geometries(geometries)
        
