import os 
import time
import rosbag
import rospy
import yaml 
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import json
import cv_bridge
import cv2
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import TransformStamped
import ros_numpy
from vgn.perception import Transform, CameraIntrinsic
from vgn.perception import ScalableTSDFVolume
import graspnetAPI
from grasp_detection.msg import Grasp, Perception, PerceptionSingleCamera, BoundingBox2D, BoundingBox3D
from grasp_detection.srv import DetectGrasps, DetectGraspsRequest, DetectGraspsResponse
from detectors.utils import data_to_percetion_msg, perform_icp_registration, open3d_frustum_filter
from detectors.detector_anygrasp import DetectorAnygrasp
from detectors.config import ConfigAnygrasp

def get_closest_message(bag, topic, timestamp: rospy.Time, delta=0.2):
    closest_msg = None
    closest_t = None

    start_time = rospy.Time(timestamp.to_sec() - delta)
    end_time = rospy.Time(timestamp.to_sec() + delta)

    for topic, msg, t in bag.read_messages(topics=[topic], start_time=start_time, end_time=end_time):
        if closest_t is None or abs(t.to_sec() - timestamp.to_sec()) < abs(closest_t.to_sec() - timestamp.to_sec()):
            closest_msg = msg
            closest_t = t
    return closest_msg, closest_t

def inverse_transform(rotation, translation):
    # Create Rotation object from given rotation
    rotation = Rotation.from_quat(rotation)
    
    # Calculate inverse rotation
    inv_rotation = rotation.inv()
    
    # Calculate inverse translation
    inv_translation = -1 * np.dot(inv_rotation.as_matrix(), translation)

    # Return as a dictionary
    return inv_rotation.as_quat(), inv_translation

class RosbagData:

    def __init__(self, bag_file, camera_config_file):
        self.bag = rosbag.Bag(bag_file)
        self.camera_config = self.read_camera_config(camera_config_file)
        self.camera_names = ['camera_left', 'camera_right', 'camera_top']
        self.rgb_topics = [f'/{name}/color/image_raw' for name in self.camera_names]
        self.depth_topics = [f'/{name}/aligned_depth_to_color/image_raw' for name in self.camera_names]
        self.pcl_topics = [f'/{name}/depth/color/points' for name in self.camera_names]
        # use aligned_depth, so camera intrinsic is color camera
        self.intrinsics_topics = [f'/{name}/aligned_depth_to_color/camera_info' for name in self.camera_names]
        # self.extrinsics_topics = [f'/{name}/depth_camera_pose' for name in self.camera_names]
        self.bridge = cv_bridge.CvBridge()


    def get_data(self, t):
                    
        rgb_msgs = {topic: get_closest_message(self.bag, topic, t)[0] for topic in self.rgb_topics}
        depth_msgs = {topic: get_closest_message(self.bag, topic, t)[0] for topic in self.depth_topics}
        pcl_msgs = {topic: get_closest_message(self.bag, topic, t)[0] for topic in self.pcl_topics}
        intrinsics_msgs = {topic: get_closest_message(self.bag, topic, t)[0] for topic in self.intrinsics_topics}

        rgb_images = []
        depth_images = []
        point_clouds = []
        intrinsics = []
        extrinsics = []

        for topic in self.rgb_topics:
            msg = rgb_msgs[topic]
            rgb_images.append(self.rgb_from_msg(msg))

        for topic in self.depth_topics:
            msg = depth_msgs[topic]
            depth_images.append(self.depth_from_msg(msg))

        for topic in self.pcl_topics:
            msg = pcl_msgs[topic]
            point_clouds.append(self.pcl_from_msg(msg))

        for topic in self.intrinsics_topics:
            msg = intrinsics_msgs[topic]
            intrinsics.append(self.intrinsic_from_msg(msg))

        # read extrinsics from config file
        for camera_name in self.camera_names:
            camera_config = self.camera_config[camera_name]
            translation = camera_config['translation']
            rotation = camera_config['rotation']
            # FIXME: the collected extrinsics are the inverse of the actual extrinsics
            inv_rotation, inv_translation = inverse_transform(rotation, translation)
            # NOTE: transform camera extrinsic from world frame to robot base frame 
            tf_cam2world = Transform.from_dict({
                'rotation': np.array(inv_rotation),
                'translation': np.array(inv_translation)
            })
            tf_robot_base2world = Transform(
                rotation=Rotation.from_euler('z', 45, degrees=True),
                translation=np.array([0.29894367196983385, -0.3565493252919476, -0.014020232348223055])
            )
            tf_world2robot_base = tf_robot_base2world.inverse()
            tf_cam2robot_base = tf_cam2world * tf_world2robot_base
            extrinsics.append(tf_cam2robot_base)

        data = {
            'camera_names': self.camera_names, # ['camera_left', 'camera_right', 'camera_top']
            'rgb_camera_frame_list': [f'{name}_color_optical_frame' for name in self.camera_names],
            'depth_camera_frame_list': [f'{name}_depth_optical_frame' for name in self.camera_names],
            'rgb_image_list': rgb_images,
            'depth_image_list': depth_images, 
            # 'points': point_clouds,
            # color and depth already aligned 
            'rgb_camera_intrinsic_list': intrinsics,
            'rgb_camera_extrinsic_list': extrinsics,
            'depth_camera_intrinsic_list': intrinsics,
            'depth_camera_extrinsic_list': extrinsics
        }

        return data

    def read_camera_config(self, config_file):
        """
        camera_1:
            name: camera_right
            serial_number: 141722070059
            parent_link: tag_3
            translation: [0.707, -0.736, 0.617]
            rotation: [0.832, 0.243, -0.224, -0.446]

        camera_2:
            name: camera_left
            serial_number: 134722070628
            parent_link: tag_3
            translation: [-0.569, -0.738, 0.631]
            rotation: [0.812, -0.299, 0.189, -0.464]
        
        camera_3:
            name: camera_top
            serial_number: 213622078748
            parent_link: tag_3
            translation: [-0.110, -0.191, 1.169]
            rotation: [0.990, -0.028, 0.135, 0.031]
        """
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
            # convert it to python dictionary with {name: ...}
            config = {camera['name']: camera for _, camera in config.items()}
        return config

    def rgb_from_msg(self, msg):
        # Extract RGB image from message
        return self.bridge.imgmsg_to_cv2(msg, "rgb8")
    
    def depth_from_msg(self, msg):
        # Extract depth image from message
        depth_image_np = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        # realsense depth image is uint16 in millimeter
        depth_image_np = depth_image_np.astype(np.float32) / 1000.0
        return depth_image_np

    def pcl_from_msg(self, msg):
        # Extract point cloud from message
        pcl_numpy = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        return pcl_numpy

    def intrinsic_from_msg(self, msg: CameraInfo):
        # Extract camera intrinsics from message

        intrinsic = CameraIntrinsic(
            width=msg.width,
            height=msg.height,
            fx=msg.K[0],   
            fy=msg.K[4],
            cx=msg.K[2],
            cy=msg.K[5],
        )
        return intrinsic


if __name__ == "__main__":
    # bag_file = '/home/junting/Documents/rosbag/my_data.bag'
    # bag_file = '/home/junting/Documents/rosbag/triple_persimmon.bag'
    bag_file = '/home/junting/Documents/rosbag/demo_1.bag'
    camera_config_file = '/home/junting/anygrasp_ws/src/grasp_detection/src/camera_params.yaml'
    bag = RosbagData(bag_file, camera_config_file)
    vis_3d_integration = True
    vis_2d_detection = False

    rospy.init_node('send_perception_data', anonymous=False)

    # get bag time range
    bag_info = bag.bag.get_type_and_topic_info()
    print(bag_info)

    bag_reader = RosbagData(bag_file, camera_config_file)
    # select a time stamp in the middle of the bag
    t = rospy.Time(0.5 * (bag.bag.get_end_time() + bag.bag.get_start_time()))
    data = bag_reader.get_data(t)
    print(data.keys())

    ####################### Perform open3d ICP registration ###########################
    # corrected_extrinsics_dict = perform_icp_registration(data, depth_trunc=1.5, source_camera='camera_top')

    # visualize 3D grasps in 3D integration
    if False:
        tsdf = ScalableTSDFVolume(
            size=0.5, # dummy
            resolution=100, # dummy
            voxel_size=0.001,
            color_type='rgb'
        )
        # integrate rgb/depth images 
        for camera_name in bag_reader.camera_names:
            # get rgb/depth images and camera intrinsics
            rgb_image = data['rgb_image_list'][bag_reader.camera_names.index(camera_name)]
            depth_image = data['depth_image_list'][bag_reader.camera_names.index(camera_name)]
            depth_camera_intrinsic = data['depth_camera_intrinsic_list'][bag_reader.camera_names.index(camera_name)]
            depth_camera_extrinsic = data['depth_camera_extrinsic_list'][bag_reader.camera_names.index(camera_name)]
            # integrate
            tsdf.integrate(
                rgb_img=rgb_image,
                depth_img=depth_image,
                intrinsic=depth_camera_intrinsic,
                extrinsic=depth_camera_extrinsic
            )
        # get point cloud
        point_cloud = tsdf.get_cloud()
        # coords frame 
        coords = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        # visualize 3D grasps
        # grippers = gg.to_open3d_geometry_list()
        # visualize
        o3d.visualization.draw_geometries([point_cloud, coords])

    ########################### save rgb/depth images to log/{t} folder #####################
    log_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'log', str(t.to_nsec()))
    log_dir = os.path.join("/home/junting/anygrasp_ws/src/grasp_detection/", 'log', str(t.to_nsec()))
    # os.makedirs(log_dir, exist_ok=True)
    # print(f'Saving data to log_dir: {log_dir}')
    # for camera_name in bag_reader.camera_names:
    #     # save rgb image 
    #     rgb_image = data['rgb_image_list'][bag_reader.camera_names.index(camera_name)]
    #     cv2.imwrite(os.path.join(log_dir, f'{camera_name}_rgb.png'), rgb_image[ :, :, ::-1])
    #     # save depth image as 16UC1
    #     depth_image = data['depth_image_list'][bag_reader.camera_names.index(camera_name)]
    #     depth_image = (depth_image * 1000).astype(np.uint16)
    #     cv2.imwrite(os.path.join(log_dir, f'{camera_name}_depth.png'), depth_image)

    #################### start to do grasp detection ############################
    # load detections from log/{t}/detections.json
    # {
    #     "object_1": {
    #         "camera_left": [202.6961, 297.2477, 252.2399, 342.4987],
    #         "camera_right": [264.9653, 307.3395, 314.0509, 352.6347],
    #         "camera_top": [262.1277, 202.8376, 301.9899, 242.0287]
    #     },
    #     ...
    # }
    with open(os.path.join(log_dir, 'detections.json'), 'r') as f:
        detections = json.load(f)
    # object_to_detect = "persimmon_1"
    object_to_detect = "bowl"
    object_detections = detections[object_to_detect]
    
    # update detections in data dict
    width = data['rgb_image_list'][0].shape[1]
    height = data['rgb_image_list'][0].shape[0]
    data['depth_bboxes'] = [
        # (np.array(object_detections[camera_name]) / 1000.0 * np.array([width, height, width, height])).astype(int)
        np.array(object_detections[camera_name]).astype(int)
        for camera_name in data['camera_names']
    ]

    if vis_2d_detection:
        # visualize 2D detection by drawing bounding boxes on rgb images
        for camera_name in data['camera_names']:
            rgb_image = data['rgb_image_list'][data['camera_names'].index(camera_name)]
            bbox = data['depth_bboxes'][data['camera_names'].index(camera_name)]
            # draw bounding box
            cv2.rectangle(rgb_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            cv2.imshow(f'{camera_name}_rgb', rgb_image[:, :, ::-1])
            cv2.waitKey(0)
            cv2.destroyAllWindows()


    # fill in req
    req = DetectGraspsRequest()
    req.perception_data = data_to_percetion_msg(data, bag_reader.bridge)

    # send to detection node by calling /detect_grasps service
    # detect_grasps = rospy.ServiceProxy('/detect_grasps', DetectGrasps)
    # rospy.wait_for_service('/detect_grasps')
    # res = detect_grasps(req)
    # print(res)

    detector = DetectorAnygrasp('/detect_grasps', ConfigAnygrasp())
    detector.load_model()

    gg, min_bound, max_bound = detector.detect_callback(req)

    # visualize 3D grasps in 3D integration
    if vis_3d_integration:
        tsdf = ScalableTSDFVolume(
            size=0.5, # dummy
            resolution=100, # dummy
            voxel_size=0.005,
            color_type='rgb'
        )
        # integrate rgb/depth images 
        for camera_name in bag_reader.camera_names:
            # get rgb/depth images and camera intrinsics
            rgb_image = data['rgb_image_list'][bag_reader.camera_names.index(camera_name)]
            depth_image = data['depth_image_list'][bag_reader.camera_names.index(camera_name)]
            depth_camera_intrinsic = data['depth_camera_intrinsic_list'][bag_reader.camera_names.index(camera_name)]
            depth_camera_extrinsic = data['depth_camera_extrinsic_list'][bag_reader.camera_names.index(camera_name)]
            # integrate
            tsdf.integrate(
                rgb_img=rgb_image,
                depth_img=depth_image,
                intrinsic=depth_camera_intrinsic,
                extrinsic=depth_camera_extrinsic
            )
        # get point cloud
        point_cloud = tsdf.get_cloud()
        # coords frame 
        coords = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        # visualize 3D grasps
        grippers = gg.to_open3d_geometry_list()
        # visualize
        o3d.visualization.draw_geometries([*grippers, point_cloud, coords])

    print('Done')
    rospy.spin()



    
