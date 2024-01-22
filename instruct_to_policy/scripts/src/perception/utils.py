from typing import List, Dict, Tuple
import numpy as np
import os
import scipy.spatial.transform
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import json
import cv2
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN, HDBSCAN
from cv_bridge import CvBridge
from grasp_detection.msg import Perception, PerceptionSingleCamera, BoundingBox3D, BoundingBox2D
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from .scalable_tsdf import ScalableTSDFVolume

class Rotation(scipy.spatial.transform.Rotation):
    @classmethod
    def identity(cls):
        return cls.from_quat([0.0, 0.0, 0.0, 1.0])

class Transform(object):
    """Rigid spatial transform between coordinate systems in 3D space.

    Attributes:
        rotation (scipy.spatial.transform.Rotation)
        translation (np.ndarray)
    """

    def __init__(self, rotation, translation):
        assert isinstance(rotation, scipy.spatial.transform.Rotation)
        assert isinstance(translation, (np.ndarray, list))

        self.rotation = rotation
        self.translation = np.asarray(translation, np.double)

    def as_matrix(self):
        """Represent as a 4x4 matrix."""
        return np.vstack(
            (np.c_[self.rotation.as_matrix(), self.translation], [0.0, 0.0, 0.0, 1.0])
        )

    def to_dict(self):
        """Serialize Transform object into a dictionary."""
        return {
            "rotation": self.rotation.as_quat().tolist(),
            "translation": self.translation.tolist(),
        }

    def to_list(self):
        return np.r_[self.rotation.as_quat(), self.translation]

    def __mul__(self, other):
        """Compose this transform with another."""
        rotation = self.rotation * other.rotation
        translation = self.rotation.apply(other.translation) + self.translation
        return self.__class__(rotation, translation)

    def transform_point(self, point):
        return self.rotation.apply(point) + self.translation

    def transform_vector(self, vector):
        return self.rotation.apply(vector)

    def inverse(self):
        """Compute the inverse of this transform."""
        rotation = self.rotation.inv()
        translation = -rotation.apply(self.translation)
        return self.__class__(rotation, translation)

    @classmethod
    def from_matrix(cls, m):
        """Initialize from a 4x4 matrix."""
        rotation = Rotation.from_matrix(m[:3, :3])
        translation = m[:3, 3]
        return cls(rotation, translation)

    @classmethod
    def from_dict(cls, dictionary):
        rotation = Rotation.from_quat(dictionary["rotation"])
        translation = np.asarray(dictionary["translation"])
        return cls(rotation, translation)

    @classmethod
    def from_list(cls, list):
        rotation = Rotation.from_quat(list[:4])
        translation = list[4:]
        return cls(rotation, translation)

    @classmethod
    def identity(cls):
        """Initialize with the identity transformation."""
        rotation = Rotation.from_quat([0.0, 0.0, 0.0, 1.0])
        translation = np.array([0.0, 0.0, 0.0])
        return cls(rotation, translation)

    @classmethod
    def look_at(cls, eye, center, up):
        """Initialize with a LookAt matrix.

        Returns:
            T_eye_ref, the transform from camera to the reference frame, w.r.t.
            which the input arguments were defined.
        """
        eye = np.asarray(eye)
        center = np.asarray(center)

        forward = center - eye
        forward /= np.linalg.norm(forward)

        right = np.cross(forward, up)
        right /= np.linalg.norm(right)

        up = np.asarray(up) / np.linalg.norm(up)
        up = np.cross(right, forward)

        m = np.eye(4, 4)
        m[:3, 0] = right
        m[:3, 1] = -up
        m[:3, 2] = forward
        m[:3, 3] = eye

        return cls.from_matrix(m).inverse()

class CameraIntrinsic(object):
    """Intrinsic parameters of a pinhole camera model.

    Attributes:
        width (int): The width in pixels of the camera.
        height(int): The height in pixels of the camera.
        K: The intrinsic camera matrix.
    """

    def __init__(self, width, height, fx, fy, cx, cy):
        self.width = width
        self.height = height
        self.K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

    @property
    def fx(self):
        return self.K[0, 0]

    @property
    def fy(self):
        return self.K[1, 1]

    @property
    def cx(self):
        return self.K[0, 2]

    @property
    def cy(self):
        return self.K[1, 2]

    def to_dict(self):
        """Serialize intrinsic parameters to a dict object."""
        data = {
            "width": self.width,
            "height": self.height,
            "K": self.K.flatten().tolist(),
        }
        return data

    @classmethod
    def from_dict(cls, data):
        """Deserialize intrinisic parameters from a dict object."""
        intrinsic = cls(
            width=data["width"],
            height=data["height"],
            fx=data["K"][0],
            fy=data["K"][4],
            cx=data["K"][2],
            cy=data["K"][5],
        )
        return intrinsic

################################
# Geometric utils 
################################

def get_mask_from_3D_bbox(bbox_center:np.ndarray, bbox_size:np.ndarray, depth_image:np.ndarray, 
                          intrinsic:CameraIntrinsic, extrinsic:Transform)->np.ndarray:
    """
    Get 2D mask image of same size as depth image by projecting 3D bounding box onto the depth image 
    """
    # get the 8 corners of the 3D bounding box
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

    # create mask image
    mask = np.zeros(depth_image.shape, dtype=np.uint8)
    mask[min_y:max_y, min_x:max_x] = 1
    return mask
   
def get_mask_from_2D_bbox(bbox:np.ndarray, depth_image:np.ndarray)->np.ndarray:
    """
    Get 2D mask image of same size as depth image from 2D bounding box
    @param bbox: 2D bounding box in the form of [min_x, min_y, max_x, max_y]
    @param depth_image: depth image of the scene
    """
    # create empty mask image and fill in the 2D bounding box with 1 
    mask = np.zeros(depth_image.shape, dtype=np.uint8)
    mask[bbox[1]:bbox[3], bbox[0]:bbox[2]] = 1
    return mask

def integrate_point_cloud_in_bbox(intrinsics: List[np.array], extrinsics: List[np.array], depth_images_list: List[np.array], bboxes_2d_list: List[List[np.array]]) -> np.array:
    '''
    Utility to integrate point cloud from all cameras with bounding boxes as masks
    '''
    tsdf = ScalableTSDFVolume
    for camera_idx, bboxes_2d in enumerate(bboxes_2d_list):
        # Create mask image for the current camera
        mask = np.zeros(depth_images_list[camera_idx].shape, dtype=np.uint8)
        for bbox in bboxes_2d:
            mask[bbox[1]:bbox[3], bbox[0]:bbox[2]] = 1

        # Integrate point cloud from all cameras with bounding boxes as masks
        if camera_idx == 0:
            integrated_point_cloud = depth_images_list[camera_idx] * mask
        else:
            integrated_point_cloud += depth_images_list[camera_idx] * mask
    
def project_3d_to_2d(points_3d: np.array, intrinsic: CameraIntrinsic, extrinsic: Transform) -> np.array:
    """
    Project 3D points to 2D image plane
    @param points_3d: 3D points in the form of (N, 3)
    @param intrinsic: camera intrinsic parameters
    @param extrinsic: camera extrinsic parameters
    @return: 2D points in the form of (N, 2)
    """
    points_3d_cam = extrinsic.transform_point(points_3d)
    K = intrinsic.K 
    points_2d = K.dot(points_3d_cam.T).T
    points_2d = points_2d[:, :2] / points_2d[:, 2:]
    return points_2d

def is_point_in_bbox(point: np.array, bbox: np.array) -> bool:
    x, y = point
    x_min, y_min, x_max, y_max = bbox
    return x_min <= x <= x_max and y_min <= y <= y_max

def check_points_in_bbox(points_2d: np.array, bbox: np.array) -> np.array:
    xs, ys = points_2d[:, 0], points_2d[:, 1]
    x_min, y_min, x_max, y_max = bbox
    return np.logical_and(np.logical_and(x_min <= xs, xs <= x_max), np.logical_and(y_min <= ys, ys <= y_max))

def one_hot_iou(one_hot_vector_1: np.array, one_hot_vector_2: np.array) -> float:
    """
    Calculate the IOU between two one-hot vectors
    """
    intersection = np.sum(np.logical_and(one_hot_vector_1, one_hot_vector_2))
    union = np.sum(np.logical_or(one_hot_vector_1, one_hot_vector_2))
    return intersection / union

#########################
# Open 3D utils 
#########################

def open3d_frustum_filter(pcl: o3d.geometry.PointCloud, bbox_2d_list: List[np.ndarray], 
                          camera_intrinsic_list: List[CameraIntrinsic], camera_extrinsic_list: List[Transform]):
    """
    Filter open3d point cloud with frustum filters by projecting 3D points onto 2D image planes 
    and checking if they are within the 2D bounding boxes.
    @param pcl: open3d point cloud
    @param bbox_2d_list: list of 2D bounding boxes in the form of [min_x, min_y, max_x, max_y]
    @param camera_intrinsic_list: list of camera intrinsic parameters
    @param camera_extrinsic_list: list of camera extrinsic parameters
    """
    mask_list = []
    
    for bbox, intrinsic, extrinsic in zip(bbox_2d_list, camera_intrinsic_list, camera_extrinsic_list):
        if len(bbox) == 0:
            # if the bbox is empty, skip it
            continue
        # project 3D points onto 2D image plane 
        points = extrinsic.transform_point(pcl.points)
        K = intrinsic.K 
        points = K.dot(points.T).T
        pixels = (points[:, :2] / points[:, 2:]).astype(np.int32) # (x,y) pixel coordinates
        
        # check if projected pixels are within 2D bounding box
        mask = np.all(np.logical_and(pixels >= bbox[:2], pixels <= bbox[2:]), axis=1)
        mask_list.append(mask)
        
    # combine masks from all cameras
    mask = np.all(mask_list, axis=0)
    
    # create new point cloud with filtered points
    filtered_pcl = o3d.geometry.PointCloud()
    filtered_pcl.points = o3d.utility.Vector3dVector(np.asarray(pcl.points)[mask])
    filtered_pcl.colors = o3d.utility.Vector3dVector(np.asarray(pcl.colors)[mask])
    return filtered_pcl, mask

def match_bboxes_clustering(bboxes_2d_list: List[List[np.array]], intrinsics: List[np.array], extrinsics: List[np.array],
                            pcl: o3d.geometry.PointCloud, downsample=False, downsample_voxel_size=0.02,
                            min_samples=10, cluster_eps=0.02, debug_visualize=False) -> List[Tuple[np.array]]:
    '''
    Match 2D bounding boxes by 1) clustering 3D points into object instances. 2)Then match the clusters and 2D bounding boxes by projecting 
    3D points to 2D image planes and checking which cluster has the most points inside the bounding boxes.
    
    Args: 
        bboxes_2d_list: List of N lists of 2D bounding boxes in the form of [min_x, min_y, max_x, max_y] under N camera views
        intrinsics: List of N camera intrinsic parameters
        extrinsics: List of N scamera extrinsic parameters
        pcl: Open3D point cloud. The point cloud should be filtered with frustum filters to reduce computation cost! 
        downsample: Whether to downsample the point cloud before matching bboxes
        downsample_voxel_size: Voxel size for downsampling the point cloud before matching bboxes
        min_cluster_size: Minimum number of points that should be considered as a cluster
        min_samples: Minimum number of points that should be considered as a core point
        cluster_eps: Maximum distance between two points to be considered as in the same neighborhood
    Returns:
        List of tuples of matched 2D bounding boxes index in the form of (view_1_bbox_idx, view_2_bbox_idx, ..., view_N_bbox_idx). If no bbox is matched, its index will be -1.
    '''
    # Downsample the point cloud to reduce computation cost if needed
    if downsample:
        pcl = pcl.voxel_down_sample(downsample_voxel_size)
    num_views = len(bboxes_2d_list)
    
    # cluster pcl points into 
    points_3d = np.asarray(pcl.points)
    clustering = DBSCAN(eps=cluster_eps, min_samples=min_samples).fit(points_3d)
    cluster_labels = clustering.labels_
    cluster_labels_unique = np.unique(cluster_labels)
    num_clusters = len(cluster_labels_unique)
    
    # visualize the clustering result
    if debug_visualize:
        colors = plt.get_cmap("tab20")(cluster_labels_unique)
        colors[cluster_labels_unique == -1] = [0, 0, 0, 1]
        pcl.colors = o3d.utility.Vector3dVector(colors[:, :3])
        o3d.visualization.draw_geometries([pcl])
    
    # Project all points into the 2D bounding boxes and find the most appearing cluster label
    matched_bboxes_idx_tuple_list = []
    for view_idx, (intrinsic, extrinsic, bboxes) in enumerate(zip(intrinsics, extrinsics, bboxes_2d_list)):
        # project all points into the 2D bounding box
        projected_points = project_3d_to_2d(points_3d, intrinsic, extrinsic)
        
        # count the occurrences of each cluster label within the bounding box
        cluster_counts = np.zeros(num_clusters)
        for bbox in bboxes:
            mask = check_points_in_bbox(projected_points, bbox)
            bbox_cluster_labels = cluster_labels[mask]
            unique_labels, label_counts = np.unique(bbox_cluster_labels, return_counts=True)
            cluster_counts[unique_labels] += label_counts
        
        # find the most appearing cluster label
        max_count_label = np.argmax(cluster_counts)
        
        # assign the bounding box to the cluster with the most appearing label
        matched_bboxes_idx_tuple = [-1 for _ in range(num_views)]
        for bbox_idx, bbox in enumerate(bboxes):
            mask = check_points_in_bbox(projected_points, bbox)
            bbox_cluster_labels = cluster_labels[mask]
            if max_count_label in bbox_cluster_labels:
                matched_bboxes_idx_tuple[view_idx] = bbox_idx
                break
        
        # append the matched_bboxes_idx_tuple to the matched_bboxes_idx_tuple_list
        matched_bboxes_idx_tuple_list.append(tuple(matched_bboxes_idx_tuple))
    
    return matched_bboxes_idx_tuple_list
            
def match_bboxes_points_overlapping(bboxes_2d_list: List[List[np.array]], intrinsics: List[np.array], extrinsics: List[np.array], 
                 pcl: o3d.geometry.PointCloud, downsample=True, downsample_voxel_size=0.02,
                 min_match_th=0.1, debug_visualize=False) -> List[Tuple[np.array]]:
    '''
    Match 2D bounding boxes by projecting 3D points to 2D image planes and checking which points are inside the bounding boxes.
    The match between two 2D bounding boxes is evaluated by the one-hot IOU between the one-hot vectors of all 2D points that are inside the bounding boxes.
    
    Args: 
        bboxes_2d_list: List of N lists of 2D bounding boxes in the form of [min_x, min_y, max_x, max_y] under N camera views
        intrinsics: List of N camera intrinsic parameters
        extrinsics: List of N scamera extrinsic parameters
        pcl: Open3D point cloud. The point cloud should be filtered with frustum filters to reduce computation cost! 
        downsample: Whether to downsample the point cloud before matching bboxes
        downsample_voxel_size: Voxel size for downsampling the point cloud before matching bboxes
        min_match_th: Minimum one-hot IOU threshold that should be considered as a match
    Return:
        List of tuples of matched 2D bounding boxes index in the form of (view_1_bbox_idx, view_2_bbox_idx, ..., view_N_bbox_idx). If no bbox is matched, its index will be -1.
    
    Algorithm:
    - (Optional) Downsample the point cloud to reduce computation cost if needed
    - Project 3D points to 2D image planes for each camera
    - For each 2D bounding box, collect one-hot vector of all 2D points that are inside the bounding box
    - For each pair of views:
        - Calculate the one-hot IOU between one-hot vectors of all 2D bounding boxes across different camera views
        - Match 2D bounding boxes across different camera views by comparing one-hot vectors
    
    Side notes: The overall matching problem can be defined as
    Given a graph G={V,E}. The vertices are separated into N sets of exclusive vertices. Compute a set of edges connecting vertices to multiple connected components so that:
    - Each connected component contains at most one vertice from a set. In other words, any connected component should not have two or more vertices from a same vertice set. 
    - The total sum of edges scores should be maximized.
    which is a NP-hard problem. 
    So we use a greedy algorithm to solve this problem.
    '''
    # Downsample the point cloud to reduce computation cost if needed
    if downsample:
        pcl = pcl.voxel_down_sample(downsample_voxel_size)
    num_views = len(bboxes_2d_list)
    
    # Project 3D points to 2D image planes for each camera
    # NOTE: there will be invalid 2d points that are outside of the image plane, fill them with out in next step
    points_2d_list = []
    for intrinsic, extrinsic in zip(intrinsics, extrinsics):
        points_2d = project_3d_to_2d(np.asarray(pcl.points), intrinsic, extrinsic)
        points_2d_list.append(points_2d)
    
    # For each 2D bounding box, collect one-hot vector of all 2D points that are inside the bounding box
    one_hot_vectors_list = []
    for view_idx, bboxes_2d in enumerate(bboxes_2d_list): # for each view 
        points_2d = points_2d_list[view_idx]
        one_hot_vectors = []
        # for each bbox in the view
        for bbox in bboxes_2d: 
            # get one-hot vector of all 2D points that are inside the bounding box
            mask = check_points_in_bbox(points_2d, bbox)
            one_hot_vector = mask.astype(np.int32)
            one_hot_vectors.append(one_hot_vector)  
        one_hot_vectors_list.append(one_hot_vectors)    
    
    # Match 2D bounding boxes across different camera views by one-hot vector IOU
    # NOTE: The overall matching problem is a NP-hard problem. So we use a greedy algorithm to solve this problem.
    # For each view, we greedily match the bbox with the highest matching score. 
    # One bbox can only be matched once.
    # Then we remove the matched bbox and repeat the process until all bboxes are matched.
    matched_bboxes_idx_tuple_list = []
    used_bbox_idx_list = [[] for _ in range(num_views)]
    for view_idx in range(num_views): # main loop for each view   
        # get one-hot vectors of all bboxes in this view
        one_hot_vectors = one_hot_vectors_list[view_idx]
        # loop over all bboxes in this view 
        for bbox_idx in range(len(one_hot_vectors)):
            # if this bbox is already matched, skip it
            if bbox_idx in used_bbox_idx_list[view_idx]:
                continue
            # else, get the one-hot vector of this bbox and initialize the matched bbox index tuple
            one_hot_to_match = one_hot_vectors[bbox_idx] 
            matched_bboxes_idx_tuple = [-1 for _ in range(num_views)]
            matched_bboxes_idx_tuple[view_idx] = bbox_idx 
            
            # look for the best match for each of rest of the views
            # iterate over other views
            for other_view_idx in range(view_idx+1, num_views): 
        
                # match 2D bounding boxes across different camera views by selecting the bbox with the highest one-hot IOU
                # NOTE: this is a greedy algorithm, which is not guaranteed to find the optimal solution
                other_view_one_hot_vectors = one_hot_vectors_list[other_view_idx]
                best_match_bbox_idx = -1
                best_match_score = 0
                for other_view_bbox_idx, other_view_one_hot_vector in enumerate(other_view_one_hot_vectors):
                    if other_view_bbox_idx not in used_bbox_idx_list[other_view_idx]: # check if the bbox is already matched
                        # calculate one-hot IOU
                        match_score = one_hot_iou(one_hot_to_match, other_view_one_hot_vector)
                        # if the match score is higher than the current best match score and min_match_th, update the best match
                        if match_score > best_match_score and match_score > min_match_th:
                            best_match_score = match_score
                            best_match_bbox_idx = other_view_bbox_idx

                # if a match is found, update the matched_bboxes_idx_tuple and used_bbox_idx_list
                if best_match_bbox_idx != -1:
                    matched_bboxes_idx_tuple[other_view_idx] = best_match_bbox_idx
                    used_bbox_idx_list[other_view_idx].append(best_match_bbox_idx)
            
            # append the matched_bboxes_idx_tuple to the matched_bboxes_idx_tuple_list
            matched_bboxes_idx_tuple_list.append(tuple(matched_bboxes_idx_tuple))
            
    return matched_bboxes_idx_tuple_list

#########################
# Data utils 
#########################
       
def data_to_percetion_msg(data: Dict, bridge:CvBridge)->Perception:        
    """
    # TODO: expand to multiple detections data if needed 
    Convert data dictionary to Perception ROS message
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
        'detections_list':[
            {'<object_id>': [min_x, min_y, max_x, max_y]},
        ],
        'bboxes_3d_dict': {
            '<object_id>': {'center': [x, y, z], 'size': [x, y, z]}
        }
    }
    """

    perception_msg = Perception(
        header = Header(frame_id="world"),
    )
    
    if 'bboxes_3d_dict' in data:
        if len(data['bboxes_3d_dict']) > 0:
            perception_msg.bboxes_3d = []
            # currently only axis-aligned bounding box is supported
            for object_id, bbox_3d in data['bboxes_3d_dict'].items():
                perception_msg.bboxes_3d.append(BoundingBox3D(
                    object_id = object_id,
                    center = Pose(
                        position = Point(*bbox_3d['center']),
                        orientation = Quaternion(0, 0, 0, 1)
                    ),
                    size = Vector3(*bbox_3d['size'])
                ))
    
    # fill camera data 
    for i, name in enumerate(data['camera_names']):
        camera_data = PerceptionSingleCamera()
        camera_data.camera_id = name
        # convert numpy array to ROS Image message
        camera_data.rgb_image = bridge.cv2_to_imgmsg(data['rgb_image_list'][i], encoding="rgb8")
        camera_data.depth_image = bridge.cv2_to_imgmsg(data['depth_image_list'][i], encoding="passthrough")
        
        # fill camera info 
        camera_data.rgb_camera_info = CameraInfo(
            header = Header(frame_id=data['rgb_camera_frame_list'][i]),
            width = data['rgb_image_list'][i].shape[1],
            height = data['rgb_image_list'][i].shape[0],
            K = data['rgb_camera_intrinsic_list'][i].K.flatten().tolist()
        )
        camera_data.depth_camera_info = CameraInfo(
            header = Header(frame_id=data['depth_camera_frame_list'][i]),
            width = data['depth_image_list'][i].shape[1],
            height = data['depth_image_list'][i].shape[0],
            K = data['depth_camera_intrinsic_list'][i].K.flatten().tolist()
        )  
        
        # fill camera pose
        camera_data.rgb_camera_pose = Pose(
            position = Point(*data['rgb_camera_extrinsic_list'][i].translation),
            orientation = Quaternion(*data['rgb_camera_extrinsic_list'][i].rotation.as_quat())
        )
        
        camera_data.depth_camera_pose = Pose(
            position = Point(*data['depth_camera_extrinsic_list'][i].translation),
            orientation = Quaternion(*data['depth_camera_extrinsic_list'][i].rotation.as_quat())
        )
        
        # add 2D bounding box if available 
        if "detections_list" in data:
            camera_data.detections = []
            detections = data["detections_list"][i]
            for object_id, bbox_2d in detections.items():
                camera_data.detections.append(BoundingBox2D(
                    object_id = object_id,
                    bbox = bbox_2d
                ))
            
        perception_msg.cameras_data.append(camera_data)
        
    return perception_msg

def camera_on_sphere(origin, radius, theta, phi):
    eye = np.r_[
        radius * np.sin(theta) * np.cos(phi),
        radius * np.sin(theta) * np.sin(phi),
        radius * np.cos(theta),
    ]
    target = np.array([0.0, 0.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])  # this breaks when looking straight down
    return Transform.look_at(eye, target, up) * origin.inverse()

def load_data_from_multiview_detection(data_dir, world_name, exclude_keywords=[]): 
    """
    Load data from multiview detection dataset
    
    Args:
        data_dir: path to the dataset
        world_name: name of the world
        exclude_keywords: list of keywords to exclude from the dataset
    Returns:
        data: Dictionary of all data.
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
            'detections_list':[
                {'<object_id>': <2d bounding box>}, # Dictionary of 2D bounding boxes in the form of [min_x, min_y, max_x, max_y]
            ],
        }    
    """
    # define data paths
    annot_file = os.path.join(data_dir, 'annotations', f"{world_name}.json")
    rgb_dir = os.path.join(data_dir, 'rgb_images')
    depth_dir = os.path.join(data_dir, 'depth_images')
    # return lists
    instriinsics_list = []
    extrinsics_list = []
    detections_list = []
    rgb_image_list = []
    depth_image_list = []
    # load camera info and detections from annotations
    with open(annot_file, 'r') as f:
        detections_data = json.load(f)
        camera_names = list(detections_data.keys())
            
    for camera_name in camera_names:
        # load camera intrinsic
        intrinsic = CameraIntrinsic(
            detections_data[camera_name]['camera_info']['width'],
            detections_data[camera_name]['camera_info']['height'],
            detections_data[camera_name]['camera_info']['K'][0],
            detections_data[camera_name]['camera_info']['K'][4],
            detections_data[camera_name]['camera_info']['K'][2],
            detections_data[camera_name]['camera_info']['K'][5],
        )
        instriinsics_list.append(intrinsic)

        # load camera extrinsic
        quat = detections_data[camera_name]['camera_info']['rotation']
        trans = detections_data[camera_name]['camera_info']['translation']
        extrinsic = Transform(
            rotation=R.from_quat(quat),
            translation=np.array(trans)
        )
        extrinsics_list.append(extrinsic)

        # load 2D bounding boxes
        detections = {}
        for label, bbox in detections_data[camera_name]['detections'].items():
            # skip object with certain key words and empty detections
            if any([kw in label for kw in exclude_keywords]) or len(bbox) == 0:
                continue
            detections[label] = bbox
        detections_list.append(detections)
        
        # load images
        # rgb image
        rgb_image = cv2.imread(os.path.join(rgb_dir, world_name + '_' + camera_name + '.png'), cv2.IMREAD_COLOR)[..., ::-1].copy()
        rgb_image_list.append(rgb_image)
        
        # depth image
        depth_image = cv2.imread(os.path.join(depth_dir, world_name + '_' + camera_name + '.png'), cv2.IMREAD_UNCHANGED)
        # convert 16UC1 to 32FC1 
        depth_image = depth_image.astype(np.float32) / 1000.0
        depth_image_list.append(depth_image)
        
    # put data into dictionary
    data = {
        'camera_names': camera_names,
        'rgb_image_list': rgb_image_list,
        'rgb_camera_intrinsic_list': instriinsics_list,
        'rgb_camera_frame_list': ['world' for _ in range(len(camera_names))], # always use world frame
        'rgb_camera_extrinsic_list': extrinsics_list,
        'depth_image_list': depth_image_list,
        'depth_camera_intrinsic_list': instriinsics_list,
        'depth_camera_frame_list': ['world' for _ in range(len(camera_names))], # always use world frame
        'depth_camera_extrinsic_list': extrinsics_list,
        'detections_list': detections_list,
    }
    return data

################################
#  Visualization utils
################################

def draw_bboxes_2d(rgb_image: np.ndarray, bboxes_2d: List[np.ndarray], color=(0, 255, 0), thickness=5) -> np.ndarray:
    """
    Draw 2D bounding boxes on the rgb image

    Args:
        rgb_image (np.ndarray): RGB image.
        bboxes_2d (List[np.ndarray]): List of 2D bounding boxes in the form of [min_x, min_y, max_x, max_y].
        color (tuple, optional): Bounding box color. Defaults to (0, 255, 0).
        thickness (int, optional): Bounding box thickness. Defaults to 2.
        
    Returns:
        np.ndarray: RGB image with bounding boxes.
    """
    for bbox in bboxes_2d:
        cv2.rectangle(rgb_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color=color, thickness=thickness)
    return rgb_image


def draw_multiview_bbox_matches(rgb_image_list: List[np.ndarray], bboxes_2d_list: List[List[np.ndarray]], 
                                matched_bboxes_idx_tuple_list: List[Tuple[np.ndarray]], thickness=5) -> np.ndarray:
    """
    Draw all 2D bounding box matches across different camera views.
    Place all RGB images from top to bottom.
    For each tuple of matched bounding boxes across different views, draw them with the same color. Different tuples will have different colors.
    Then draw lines between the matched bounding boxes between top-to-bottom images.
    
    Args:
        rgb_image_list (List[np.ndarray]): List of RGB images.
        bboxes_2d_list (List[List[np.ndarray]]): List of lists of 2D bounding boxes in the form of [min_x, min_y, max_x, max_y].
        matched_bboxes_idx_tuple_list (List[Tuple[np.array]]): List of tuples of matched 2D bounding box indices in the form of (view_1_bbox_idx, view_2_bbox_idx, ..., view_N_bbox_idx). If no bbox is matched, its index will be -1.
        thickness (int): Bounding box thickness.
        
    Returns:
        np.ndarray: RGB image with bounding boxes.
    """
    num_views = len(rgb_image_list)
    max_width = max([rgb_image.shape[1] for rgb_image in rgb_image_list])
    total_height = sum([rgb_image.shape[0] for rgb_image in rgb_image_list])
    canvas = np.zeros((total_height, max_width, 3), dtype=np.uint8)
    
    offset = 0
    # first draw all RGB images
    for view_idx, rgb_image in enumerate(rgb_image_list):
        # place all RGB images from top to bottom on the canvas
        canvas[offset:offset+rgb_image.shape[0], :rgb_image.shape[1]] = rgb_image
        offset += rgb_image.shape[0]
        
    offset = 0
    # draw matched bounding boxes and lines
    for view_idx, rgb_image in enumerate(rgb_image_list):
        cmap = plt.cm.get_cmap('tab20')  # Choose a colormap
        for match_idx, matched_bboxes_idx_tuple in enumerate(matched_bboxes_idx_tuple_list):
            # select color for this match
            color = (np.array(cmap(match_idx % 20)[:3]) * 255).astype(int).tolist()
            if matched_bboxes_idx_tuple[view_idx] != -1:
                # draw bounding box
                bbox_2d = bboxes_2d_list[view_idx][matched_bboxes_idx_tuple[view_idx]].copy()  # Make a copy of the bounding box before modifying it
                bbox_2d[1] += offset
                bbox_2d[3] += offset
                cv2.rectangle(canvas, (bbox_2d[0], bbox_2d[1]), (bbox_2d[2], bbox_2d[3]), color=color, thickness=thickness)
                
                # draw line between matched bounding boxes
                if view_idx < num_views - 1 and matched_bboxes_idx_tuple[view_idx+1] != -1:
                    next_bbox_2d = bboxes_2d_list[view_idx+1][matched_bboxes_idx_tuple[view_idx+1]].copy()  # Make a copy of the bounding box before modifying it
                    next_bbox_2d[1] += offset + rgb_image.shape[0]  # Add offset for the next image
                    next_bbox_2d[3] += offset + rgb_image.shape[0]  # Add offset for the next image

                    # change the start point of the line to the middle of the current bounding box
                    start_point = ((bbox_2d[0] + bbox_2d[2]) // 2, bbox_2d[3])
                    # change the end point of the line to the middle of the next bounding box
                    end_point = ((next_bbox_2d[0] + next_bbox_2d[2]) // 2, next_bbox_2d[1])

                    cv2.line(canvas, start_point, end_point, color=color, thickness=thickness)
        offset += rgb_image.shape[0]

    return canvas

            
    