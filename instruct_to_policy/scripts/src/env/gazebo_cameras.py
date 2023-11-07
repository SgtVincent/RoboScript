import os 
from typing import List, Tuple, Dict
import numpy as np 
from collections import deque
import genpy 
import rospy
import ros_numpy
import tf2_ros

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import TransformStamped

from src.perception.utils import Transform, CameraIntrinsic

class GazeboRGBDCamera:
    """
    This class is used to buffer gazebo RGBD camera data and provide easy IO interface.
    """
    def __init__(self, camera_name, namespace="", buffer_size=10, use_aligned_depth=True, sub_pcl=False) -> None:
        self.ns = namespace
        self.camera_name = camera_name
        self.use_aligned_depth = use_aligned_depth
        self.sub_pcl = sub_pcl
        self.rgb_camera_info:CameraInfo  = None
        self.depth_camera_info:CameraInfo = None
        
        # subscribe to camera topics
        self.rgb_image_sub = rospy.Subscriber(self.ns + '/' + camera_name + "/color/image_raw", Image, self.rgb_image_callback)
        if use_aligned_depth:
            self.depth_image_sub = rospy.Subscriber(self.ns + '/' + camera_name + "/aligned_depth/image_raw", Image, self.depth_image_callback)
        else:
            self.depth_image_sub = rospy.Subscriber(self.ns + '/' + camera_name + "/depth/image_raw", Image, self.depth_image_callback)
        self.rgb_camera_info_sub = rospy.Subscriber(self.ns + '/' + camera_name + "/color/camera_info", CameraInfo, self.rgb_camera_info_callback)
        
        if use_aligned_depth:
            self.depth_camera_info_sub = rospy.Subscriber(self.ns + '/' + camera_name + "/aligned_depth/camera_info", CameraInfo, self.depth_camera_info_callback)
        else:
            self.depth_camera_info_sub = rospy.Subscriber(self.ns + '/' + camera_name + "/depth/camera_info", CameraInfo, self.depth_camera_info_callback)
        
        # point cloud subscriber 
        self.sub_pcl = None
        if sub_pcl:
            self.pcl_sub = rospy.Subscriber(self.ns + '/' + camera_name + "/depth/color/points", PointCloud2, self.pcl_callback)


        self.bridge = CvBridge()
        # buffer for data synchronization
        self.rgb_buffer = deque(maxlen=buffer_size)
        self.depth_buffer = deque(maxlen=buffer_size)
        self.pcl_buffer = deque(maxlen=buffer_size)

        # get camera info from ros
        self._init_with_camera_info()

    def _init_with_camera_info(self):
        """
        Get camera intrinsic and frame ids from camera info
        """
        while self.rgb_camera_info is None:
            rospy.loginfo(f"{self.camera_name}: Waiting for {self.camera_name}/color/camera_info...")
            rospy.sleep(0.5)
        rospy.loginfo(f"{self.camera_name}: {self.camera_name}/color/camera_info received!")

        # parse rgb camera intrinsics from camera info
        self.rgb_camera_intrinsic = CameraIntrinsic(
            width=self.rgb_camera_info.width,
            height=self.rgb_camera_info.height,
            fx=self.rgb_camera_info.K[0],   
            fy=self.rgb_camera_info.K[4],
            cx=self.rgb_camera_info.K[2],
            cy=self.rgb_camera_info.K[5],
        )
        # parse rgb camera frame id from camera info
        self.rgb_camera_frame_id = self.rgb_camera_info.header.frame_id

        while self.depth_camera_info is None:
            rospy.loginfo(f"{self.camera_name}: Waiting for {self.camera_name}/depth/camera_info...")
            rospy.sleep(0.5)
        rospy.loginfo(f"{self.camera_name}: {self.camera_name}/depth/camera_info received!")

        # parse depth camera intrinsics from camera info
        self.depth_camera_intrinsic = CameraIntrinsic(
            width=self.depth_camera_info.width,
            height=self.depth_camera_info.height,
            fx=self.depth_camera_info.K[0],   
            fy=self.depth_camera_info.K[4],
            cx=self.depth_camera_info.K[2],
            cy=self.depth_camera_info.K[5],
        )

        # parse depth camera frame id from camera info
        self.depth_camera_frame_id = self.depth_camera_info.header.frame_id
        

    def rgb_image_callback(self, data):
        try:
            # save all raw data for synchronization and processing
            self.rgb_buffer.append(data)
        except CvBridgeError as e:
            rospy.logerr(e)
    
    def depth_image_callback(self, data):
        try:
            # save all raw data for synchronization and processing
            self.depth_buffer.append(data)
        except CvBridgeError as e:
            rospy.logerr(e)

    def pcl_callback(self, data):
        try:
            # save all raw data for synchronization and processing
            self.pcl_buffer.append(data)
        except CvBridgeError as e:
            rospy.logerr(e)

    def rgb_camera_info_callback(self, data):
        self.rgb_camera_info = data

    def depth_camera_info_callback(self, data):
        self.depth_camera_info = data 
        
    def get_data_by_timestamp(self, timestamp:genpy.Time)->Tuple[np.ndarray, np.ndarray]:
        """
        Get synchronized rgb and depth image from buffer by its closest timestamp
        """
        rgb_image = self.rgb_buffer[-1]
        depth_image = self.depth_buffer[-1]
        rgb_timestamp = None
        depth_timestamp = None
        for rgb_image in self.rgb_buffer:
            rgb_timestamp = rgb_image.header.stamp
            if rgb_timestamp > timestamp:
                break
        for depth_image in self.depth_buffer:
            depth_timestamp = depth_image.header.stamp
            if depth_timestamp > timestamp:
                break
        # TODO: do we need to check if rgb_timestamp == depth_timestamp?
        try:
            rgb_image_np = self.bridge.imgmsg_to_cv2(rgb_image, "rgb8")
            depth_image_np = self.bridge.imgmsg_to_cv2(depth_image, "16UC1")
            # realsense depth image is uint16 in millimeter
            depth_image_np = depth_image_np.astype(np.float32) / 1000.0
        except CvBridgeError as e:
            rospy.logerr(e)
            return None, None
        
        return rgb_image_np, depth_image_np

    def get_pcl_by_timestamp(self, timestamp:genpy.Time)->np.ndarray:
        """
        Get synchronized point cloud from buffer by its closest timestamp
        """
        pcl:PointCloud2 = self.pcl_buffer[-1]
        pcl_timestamp = None
        for pcl in self.pcl_buffer:
            pcl_timestamp = pcl.header.stamp
            if pcl_timestamp > timestamp:
                break
        
        # return processed point cloud as numpy array
        pcl_numpy = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcl)
        return pcl_numpy
        

    def get_last_timestamp(self)->Tuple[genpy.Time, genpy.Time]:
        """
        Get last synchronized timestamp of rgb and depth image
        """
        if len(self.rgb_buffer) > 0 and len(self.depth_buffer) > 0:
            rgb_image = self.rgb_buffer[-1]
            depth_image = self.depth_buffer[-1]
            return rgb_image.header.stamp, depth_image.header.stamp
        else:
            return None, None
        
    def get_camera_instrinsics_and_frames(self)->Tuple[CameraIntrinsic, str, CameraIntrinsic, str]:
        """
        Get camera instrinsics and frames
        """
        return self.rgb_camera_intrinsic, self.rgb_camera_frame_id, self.depth_camera_intrinsic, self.depth_camera_frame_id

class GazeboRGBDCameraSet:
    """
    This class is used to buffer gazebo RGBD camera set data and provide easy IO interface.
    """
    def __init__(self, cameras_list: List[str], namespace="", buffer_size=10, use_aligned_depth=True, sub_pcl=False) -> None:
        
        self.ns = namespace
        self.buffer_size = buffer_size
        self.cameras_list = cameras_list
        self.sub_pcl = sub_pcl
        self.cameras: Dict[str, GazeboRGBDCamera] = {}
        for camera_name in cameras_list:
            self.cameras[camera_name] = GazeboRGBDCamera(camera_name, namespace=namespace, 
                                                         buffer_size=buffer_size, use_aligned_depth=use_aligned_depth, sub_pcl=sub_pcl)

        # initialize tf listener 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # waitForTransform function not needed for tf2
        # _, first_rgb_frame, _, _ = self.cameras[self.cameras_list[0]].get_camera_instrinsics_and_frames()
        # self.tf_listener.waitForTransform("world", first_rgb_frame, rospy.Time(), rospy.Duration(4.0))


    def query_extrinsic(self, time:genpy.Time, camera_frame:str, source_frame:str="world")->Transform:
        """
        Query extrinsic transform from camera to target frame
        """
        try:
            transform:TransformStamped = self.tf_buffer.lookup_transform(
                source_frame=source_frame,
                target_frame=camera_frame,
                time=time,
                timeout=rospy.Duration(1.0)
            )
            rotation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            translation = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            
            return Transform.from_dict({
                'rotation': np.array(rotation),
                'translation': np.array(translation)
            })
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return None
        

    def get_latest_data(self):
        """
        Get latest synchronized rgb and depth image from all cameras
        """
        # first get min timestamp within all lastest timestamps from cameras
        timestamp_list = []
        for camera_name in self.cameras_list:
            rgb_timestamp, depth_timestamp = self.cameras[camera_name].get_last_timestamp()
            if rgb_timestamp is None or depth_timestamp is None:
                rospy.logwarn(f"{camera_name}: No data received yet!")
                return None, None
            else:
                timestamp_list.append(rgb_timestamp)
                timestamp_list.append(depth_timestamp)
        timestamp = min(timestamp_list)

        # then get synchronized rgb and depth image from all cameras
        data = {
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
        if self.sub_pcl:
            data['points'] = []

        for camera_name in self.cameras_list:
            rgb_image, depth_image = self.cameras[camera_name].get_data_by_timestamp(timestamp)
            if rgb_image is None or depth_image is None:
                rospy.logwarn(f"{camera_name}: No data received yet!")
                return None, None
            else:
                # load data from camera 
                rgb_camera_intrinsic, rgb_camera_frame, depth_camera_intrinsic, depth_camera_frame = \
                    self.cameras[camera_name].get_camera_instrinsics_and_frames()
                data['camera_names'].append(camera_name)
                data['rgb_image_list'].append(rgb_image)
                data['rgb_camera_intrinsic_list'].append(rgb_camera_intrinsic)
                data['rgb_camera_frame_list'].append(rgb_camera_frame)
                data['depth_image_list'].append(depth_image)
                data['depth_camera_intrinsic_list'].append(depth_camera_intrinsic)
                data['depth_camera_frame_list'].append(depth_camera_frame)

                # query extrinsic transform from camera to world
                rgb_camera_extrinsic = self.query_extrinsic(timestamp, rgb_camera_frame)
                depth_camera_extrinsic = self.query_extrinsic(timestamp, depth_camera_frame)
                data['rgb_camera_extrinsic_list'].append(rgb_camera_extrinsic)
                data['depth_camera_extrinsic_list'].append(depth_camera_extrinsic)

                if self.sub_pcl:
                    points = self.cameras[camera_name].get_pcl_by_timestamp(timestamp)
                    data['points'].append(points)

        return data
    
