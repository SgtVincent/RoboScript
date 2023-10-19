import os 
import sys
import numpy as np 
from typing import List, Tuple, Dict
import open3d as o3d 
from scipy.spatial.transform import Rotation as R
import graspnetAPI 

# ROS
import rospy 
import rospkg
from grasp_detection.msg import Grasp, Perception, PerceptionSingleCamera, BoundingBox2D, BoundingBox3D
from grasp_detection.srv import DetectGrasps, DetectGraspsRequest, DetectGraspsResponse
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image, CameraInfo

# vgn utils 
from vgn.perception import TSDFVolume, ScalableTSDFVolume

# anygrasp models
# current_dir = os.path.dirname(os.path.realpath(__file__))
# anygrasp_detection_dir = os.path.join(current_dir, "anygrasp_sdk", "grasp_detection")
grasp_detection_dir = rospkg.RosPack().get_path('grasp_detection')
anygrasp_detection_dir = os.path.join(grasp_detection_dir, "src", "detectors", "anygrasp_sdk", "grasp_detection")
sys.path.append(anygrasp_detection_dir)
from gsnet import AnyGrasp
from .detector_base import DetectorBase
from .config import ConfigAnygrasp
from .utils import CameraIntrinsic, Transform, Rotation, get_mask_from_2D_bbox, open3d_frustum_filter


class DetectorAnygrasp(DetectorBase):
    def __init__(self, service_name, config):
        super().__init__(service_name, config)
        self.config: ConfigAnygrasp
        self.model = None
        self.max_grasp_num = self.config.max_grasp_num
        self.volume_type = self.config.volume_type
        self.voxel_grid_size = self.config.voxel_grid_size # not used for anygrasp since it uses point cloud as input
        self.resolution = self.config.resolution
        self.color_type = self.config.color_type
        self.voxel_size = self.config.voxel_size
        self.filter_cloud_with_bbox = self.config.filter_cloud_with_bbox
    
        self.debug = self.config.debug
        
    def load_model(self):
        self.model = AnyGrasp(self.config)
        self.model.load_net()

    def detect_callback(self, req: DetectGraspsRequest)->DetectGraspsResponse:
        """
        Callback function for the ROS service. 
        """
        # preprocess perception data: integrate depth images into TSDF volume and point cloud 
        cloud, min_bound, max_bound = self._preprocess(req.perception_data)
        
        # visualize pre-processed input 
        if self.debug:

            # create world coordinate frame
            world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=min_bound)
            # o3d.visualization.draw_geometries([*grippers, cloud, world_frame])
            o3d.visualization.draw_geometries([cloud, world_frame])
        
        # transform point cloud to the same coordinate frame as the model
        # the anygrasp model is trained with the z axis pointing down
        trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
        cloud = cloud.transform(trans_mat)
        
        points = np.asarray(cloud.points).astype(np.float32)
        colors = np.asarray(cloud.colors).astype(np.float32)
        # lims: [xmin, xmax, ymin, ymax, zmin, zmax]
        # lims = [min_bound[0], max_bound[0], min_bound[1], max_bound[1], min_bound[2], max_bound[2]]
        lims = [min_bound[0], max_bound[0], min_bound[1], max_bound[1], -max_bound[2], -min_bound[2]]
        

        
        # get prediction
        ret = self.model.get_grasp(points, colors, lims)
        if len(ret) == 2:
            gg, cloud = ret
        else:
            # if no valid grasp found, 3-tuple is returned for debugging (not used here)
            print('No Grasp detected by Anygrasp model!')
            return DetectGraspsResponse(grasps=[])
        
        gg:graspnetAPI.GraspGroup = gg.nms().sort_by_score()
        # transform grasps back to the original coordinate frame
        gg = gg.transform(trans_mat)
        gg = gg[0:self.max_grasp_num]
        
        if self.debug:
            cloud = cloud.transform(trans_mat)
            grippers = gg.to_open3d_geometry_list()

            # create world coordinate frame
            world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=min_bound)
            # o3d.visualization.draw_geometries([*grippers, cloud, world_frame])
            o3d.visualization.draw_geometries([grippers[0], cloud, world_frame])
        
        # compose response
        grasps_msg = []
        for i, grasp in enumerate(gg):
            grasp_msg = Grasp()
            grasp_msg.object_id = "" # not used yet 
            
            grasp_msg.grasp_pose = Pose()
            grasp_msg.grasp_pose.position = Point(*grasp.translation)
            grasp_msg.grasp_pose.orientation = Quaternion(*R.from_matrix(grasp.rotation_matrix).as_quat())
            
            grasp_msg.grasp_score = grasp.score
            grasp_msg.grasp_width = grasp.width
            
            grasps_msg.append(grasp_msg)

        response = DetectGraspsResponse(grasps=grasps_msg)
        return response

    def _preprocess(self, data: Perception)->Tuple[TSDFVolume, np.ndarray]:
        """
        Preprocess data for grasp detection:
        - integrate depth images into TSDF volume
        - return TSDF volume and point cloud
        """
        
        if self.volume_type == "uniform":
            tsdf = TSDFVolume(self.voxel_grid_size, self.resolution, color_type=self.color_type)
        elif self.volume_type == "scalable":
            tsdf = ScalableTSDFVolume(self.voxel_grid_size, self.resolution, color_type=self.color_type, voxel_size=self.voxel_size)

        # assume only one object detection in perception message
        bbox_list = []
        intrinsics_list = []
        extrinsics_list = []

        for i, camera_data in enumerate(data.cameras_data):
            camera_data: PerceptionSingleCamera
            rgb: Image = camera_data.rgb_image
            depth: Image = camera_data.depth_image 
            
            # convert sensor_msgs/Image
            rgb = self.cv_bridge.imgmsg_to_cv2(rgb, desired_encoding="rgb8")
            depth = self.cv_bridge.imgmsg_to_cv2(depth, desired_encoding="32FC1")
            
            camera_info_msg = camera_data.depth_camera_info
            intrinsics = CameraIntrinsic(
                width=camera_info_msg.width,
                height=camera_info_msg.height,
                fx=camera_info_msg.K[0],
                fy=camera_info_msg.K[4],
                cx=camera_info_msg.K[2],
                cy=camera_info_msg.K[5],   
            )
            
            camera_pose_msg = camera_data.depth_camera_pose
            extrinsics = Transform.from_dict(
                {
                    "translation": [
                        camera_pose_msg.position.x,
                        camera_pose_msg.position.y,
                        camera_pose_msg.position.z,
                    ],
                    "rotation": [
                        camera_pose_msg.orientation.x,
                        camera_pose_msg.orientation.y,
                        camera_pose_msg.orientation.z,
                        camera_pose_msg.orientation.w,
                    ]
                }
            )
            
            # NOTE: Assume only one detection in single request
            mask=None
            if len(camera_data.detections) > 0:
                detection:BoundingBox2D = camera_data.detections[0]
                bbox = np.array([
                    detection.x_min,
                    detection.y_min,
                    detection.x_max,
                    detection.y_max
                ])
                mask = get_mask_from_2D_bbox(bbox, depth) 

                bbox_list.append(bbox)
            else:
                bbox_list.append(None)
                
            intrinsics_list.append(intrinsics)
            extrinsics_list.append(extrinsics)

            tsdf.integrate(depth, intrinsics, extrinsics, rgb_img=rgb, mask_img=mask)


        full_cloud: o3d.geometry.PointCloud = tsdf.get_cloud()
        
        # TODO: check this filtering part is really needed
        if len(data.bboxes_3d) > 0:
            # assume only one detection in the perception message s
            bbox_msg:BoundingBox3D = data.bboxes_3d[0]
            
            # filter the cloud by the 3D bounding box
            # assume axis aligned bounding box
            bbox_3d_center = [bbox_msg.center.position.x, bbox_msg.center.position.y, bbox_msg.center.position.z]
            bbox_3d_size = [bbox_msg.size.x, bbox_msg.size.y, bbox_msg.size.z]
            
            min_bound = np.array(bbox_3d_center) - np.array(bbox_3d_size) / 2 - self.config.filter_bbox_3d_margin
            max_bound=np.array(bbox_3d_center) + np.array(bbox_3d_size) / 2 + self.config.filter_bbox_3d_margin
            
            bounding_box = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=min_bound,
                max_bound=max_bound
            )
            filtered_cloud = full_cloud.crop(bounding_box)
                
        else:
            filtered_cloud, mask = open3d_frustum_filter(full_cloud, 
                                                         bbox_list,
                                                        intrinsics_list,
                                                        extrinsics_list,
                                                        margin=self.config.filter_bbox_2d_margin)
            min_bound = filtered_cloud.get_min_bound()
            max_bound = filtered_cloud.get_max_bound()
        
        if self.filter_cloud_with_bbox:
            return filtered_cloud, min_bound, max_bound
        else:
            return full_cloud, min_bound, max_bound

        
    





