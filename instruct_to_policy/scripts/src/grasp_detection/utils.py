from typing import List, Dict, Tuple
import numpy as np
import scipy.spatial.transform
import open3d as o3d

import rospy 
from cv_bridge import CvBridge, CvBridgeError
from grasp_detection.srv import DetectGrasps, DetectGraspsRequest, DetectGraspsResponse
from grasp_detection.msg import Grasp, Perception, PerceptionSingleCamera, BoundingBox3D, BoundingBox2D
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

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

class Grasp(object):
    """Grasp parameterized as pose of a 2-finger robot hand.
    """
    # TODO: should we specify more parameters here? e.g. velocity, max force etc.
    def __init__(self, pose: Transform, width):
        self.pose = pose
        self.width = width

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
        'depth_bboxes':[],
        'bbox_3d':{
            'center': [],
            'size': [],
        },
    }
    """

    perception_msg = Perception(
        header = Header(frame_id="world"),
        # currently only axis-aligned bounding box is supported
        bboxes_3d = BoundingBox3D(
            center = Pose(
                position = Point(*data["bbox_3d"]["center"]),
                orientation = Quaternion(0, 0, 0, 1)
            ),
            size = Vector3(*data["bbox_3d"]["size"])
        )
    )
    
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
        if "depth_bboxes" in data:
            bbox = BoundingBox2D()
            bbox.object_id = ""
            bbox.min_x, bbox.min_y, bbox.max_x, bbox.max_y = data["depth_bboxes"][i]
            camera_data.detections.append(bbox)
        
        perception_msg.cameras_data.append(camera_data)
        
    return data
            
        
            
    
        
        
            
    