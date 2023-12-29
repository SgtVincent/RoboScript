#!/usr/bin/env python3
import rospy
import tf
from sensor_msgs.msg import PointCloud2
import numpy as np

import torch
from visualization_msgs.msg import Marker, MarkerArray
from visualization_msgs.msg import Marker

from grasp_executor import GraspExecutor
import rospy
from visualization_msgs.msg import Marker
import rospy
import tf

import sys
import numpy as np
import rospy
import sys
from ros_utils import get_cloud, get_grasp_marker
import matplotlib
from scipy.spatial.transform import Rotation

from open3d_ros_helper import open3d_ros_helper as orh
from std_srvs.srv import SetBool, Empty

from occupancy_prediction import models
import open3d as o3d


import argparse
import rospy
from std_msgs.msg import String
import tf
from sensor_msgs.msg import PointCloud2
import numpy as np

import ros_numpy
import torch
import open3d as o3d
from visualization_msgs.msg import Marker, MarkerArray
import sys
from visualization_msgs.msg import Marker

import pdb

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import cv_bridge
from visualization_msgs.msg import Marker
import roslib
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion

from vgn.utils.transform import Rotation, Transform

import actionlib
from vgn.perception import TSDFVolume, CameraIntrinsic

import os
import sys
import numpy as np
import yaml

from collections import OrderedDict
from typing import NamedTuple
import franka_gripper.msg
import rospy
import sys

# Brings in the SimpleActionClient
import actionlib
import time
# pdb.set_trace()
import matplotlib
from matplotlib import cm

cmap = matplotlib.cm.get_cmap("brg", 100)
inst_cmap = matplotlib.cm.get_cmap("tab10", 10)
from vgn.ConvONets.conv_onet.generation import Generator3D
from open3d_ros_helper import open3d_ros_helper as orh
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import Image



class Grasp(NamedTuple):
    orientation: np.ndarray
    position: np.ndarray
    score: float
    width: float
    instance_id: int

cmap = matplotlib.cm.get_cmap("PiYG", 100)
inst_cmap = matplotlib.cm.get_cmap("tab10", 10)


class RosGraspDetector:
    """ROS node that detects grasps and executes them.
    
    Subscribes to the pointcloud topic and publishes the grasps as markers.
    """
    
    def __init__(self) -> None:

        rospy.init_node("grasp_detector")
        # General Config
        self.base_frame = "workspace"
        self.sensor_frame = "camera_depth_optical_frame"

        self.do_grasp = True
        self.confirm_grasp = True

        self.grasp_tioggle_srv = rospy.Service("~toggle_grasp", SetBool, self.toggle_grasp_cb)

        self.is_executing = False

        self.reset_time = 0
        # self.rate = 0.4    # Hz

        self.log_meshes = True
        self.log_folder = "/data/experiments"

        self.grasp_exe = GraspExecutor(self.base_frame, reset_pose = [-0.39999520574177366, 0.4740489757251173, -0.5709029256494976, -1.1338590764153493, 0.7967397934201639, 1.0726936823311086, -1.427030496828192])
        self.grasp_exe.reset()
        self.last_ts = 0

        self.tf_listener = tf.TransformListener()

        print("Creating Model")
        self.grasp_marker_pub = rospy.Publisher(
            "/grasps", MarkerArray, queue_size=1
        )
        self.pc_pub = rospy.Publisher("/segmented_pc", PointCloud2, queue_size=1)
        self.mesh_marker_pub = rospy.Publisher("/mesh", MarkerArray, queue_size=10)
        self.current_grasp_marker = rospy.Publisher("/current_grasp", Marker, queue_size=10)
        self.cam_loc_pub = rospy.Publisher("/cam_loc", Marker, queue_size=10)

        self.cv_bridge = cv_bridge.CvBridge()
        from vgn.detection_implicit import VGNImplicit
        self.model = VGNImplicit("/home/zrene/git/DataCollection/GIGA/data/models/giga_packed.pt",
                                "giga",
                                best=True,
                                qual_th=0.5,
                                force_detection=True,
                                out_th=0.1,
                                select_top=False,
                                visualize=False)
        self.model.net.eval()
        self.net = self.model.net
   
        self.generator = Generator3D(
            self.net,
            device="cuda",
            threshold=0.5,
            input_type='pointcloud',
            padding=0,
        )
        self.img_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.img_callback, queue_size=1)
        self.reset_srv = rospy.Service("~reset", Empty, self.reset_srv_cb)
        print("*******************************************")
        print("********** Grasp Detector Ready! **********")
        print("*******************************************")


    def reset_srv_cb(self, req):
        self.grasp_exe.reset()
        return True
    
    def toggle_grasp_cb(self, req):
        self.do_grasp = req.data
        return True

    def _rate_limit(fn):
        """Decorator to block the execution of a function if the robot is moving.
        
        Sets the self.moving variable to True before executing the function and to False after.
        """
        def lock_state( self, *args, **kwargs ):
            # Check frequency
            # if (args[0].header.stamp.to_sec() - self.last_ts) < 1 / self.rate:
            #     print("Rate limitting. Skipping PC.")
            #     return
            # self.last_ts = args[0].header.stamp.to_sec()
            
            # Check blocked 
            is_executing = self.is_executing
            if is_executing:
                print("Robot is moving. Skipping data.")
                return
            try:
                ret = fn(self, *args, **kwargs)
            finally:
                self.is_executing = False
            return ret
        
        return lock_state
    
    @_rate_limit
    def img_callback(self, img_msg: PointCloud2):

        # check pointcloud delay
        if (self.reset_time - img_msg.header.stamp.to_sec()) > 0:
            print("PC delay too large. Skipping.")
            return
        # Transform the pointcloud to the base frame
        self.tf_listener.waitForTransform(
            self.base_frame, self.sensor_frame,rospy.Time(0), rospy.Duration(10.0)
        )
        cam_loc, _ = self.tf_listener.lookupTransform(
            self.base_frame, self.sensor_frame, rospy.Duration(0)
        )

    
        print("got img")

        img = self.cv_bridge.imgmsg_to_cv2(img_msg).astype(np.float32) * 0.001
        print("Converted")
        self.tf_listener.waitForTransform(
            self.base_frame, self.sensor_frame, rospy.Time(0), rospy.Duration(10.0)
        )
        cam_loc, cam_orient = self.tf_listener.lookupTransform(self.sensor_frame, self.base_frame, rospy.Duration(0))
        tsdf_vol = TSDFVolume(0.3, 40, origin = np.array([0,0,0.000]))
        cam_intrinsics = CameraIntrinsic.from_dict({
            "height": 480,
            "width": 640,
            "K":  [626.9513549804688, 0.0, 314.1402893066406, 0.0, 626.9513549804688, 244.42013549804688, 0.0, 0.0, 1.0]

        })
        cam_tf = Transform(Rotation.from_quat(cam_orient), cam_loc)

        tsdf_vol.integrate(img, cam_intrinsics, cam_tf) 
        pc = tsdf_vol.get_cloud()
        msg = orh.o3dpc_to_rospc(pc)
        msg.header.frame_id = self.base_frame

        # remove tabletop $

        self.pc_pub.publish(msg)
        pred_mesh, _ = self.generator.generate_mesh({'inputs': torch.from_numpy(tsdf_vol.get_grid()).cuda()})
        pred_mesh.vertices = (pred_mesh.vertices + 0.5) * 0.3
        import trimesh
        scene = trimesh.Scene()
        scene.add_geometry(pred_mesh)
        scene.add_geometry(trimesh.points.PointCloud(np.asarray(pc.points)))
        # o3d.visualization.draw_geometries([pred_mesh, pc])
        scene.show()
        # import pdb; pdb.set_trace()
        meshes = [pred_mesh]

        # depth_img = img
        # rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        #     o3d.geometry.Image(np.empty_like(depth_img)),
        #     o3d.geometry.Image(depth_img),
        #     depth_scale=1.0,
        #     depth_trunc=2.0,
        #     convert_rgb_to_intensity=False,
        # )
        # intrinsic = cam_intrinsics
        # extrinsic = cam_tf
        # intrinsic = o3d.camera.PinholeCameraIntrinsic(
        #     width=intrinsic.width,
        #     height=intrinsic.height,
        #     fx=intrinsic.fx,
        #     fy=intrinsic.fy,
        #     cx=intrinsic.cx,
        #     cy=intrinsic.cy,
        # )

        extrinsic = extrinsic.as_matrix()
        import pdb; pdb.set_trace()
        o3d.visualization.draw(o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic, extrinsic))
        import pdb; pdb.set_trace()

        #o3d.visualization.draw_geometries([pc])
        
        state = argparse.Namespace(tsdf=tsdf_vol, pc = pc)
        grasps, scores, _ = self.model(state)
        pts, ori = [], []
        for g in grasps:
            pts.append(g.pose.translation)
            ori.append(g.pose.rotation.as_quat())
        pts = np.array(pts)
        ori = np.array(ori)


        arr = MarkerArray()
        for id, (rot, c, s) in enumerate(zip(ori, pts, scores)):

            marker = get_grasp_marker(
                rot, c, id, color=cmap(1*(scores[id] - 0.4) + 0.4 )  if s != np.max(scores) else np.array([0,0,1,1]), frame= self.base_frame
            )

            if id == 0:
                marker.action = Marker.DELETEALL
            arr.markers.append(marker)
        self.grasp_marker_pub.publish(arr)


        # publishes meshes. currently done ugly!
        arr = MarkerArray()
        for id, mesh in enumerate(meshes):
            f = "/tmp/mesh_inst_{}.obj".format(id)
            print("Mesh saved to: ", f)
            mesh.export(f)
            marker = get_grasp_marker(
                [0, 0, 0, 1],
                [0,0,0],
                id + 1,
                color=inst_cmap(id / 10),
                mesh_resource="file://" + f,
                ns="obj_" + str(id),
                frame = self.base_frame,
            )

            if id == 0:
                marker.action = marker.DELETEALL
            arr.markers.append(marker)


            self.grasp_exe.register_object(mesh, id, [0,0,0])

        self.mesh_marker_pub.publish(arr)
        grasps_our_format = []

        for idx, g in enumerate(grasps):
            grasps_our_format.append(Grasp(g.pose.rotation.as_quat(), g.pose.translation, scores[idx],g.width, 0))




        import os;
        if self.log_meshes:
            import datetime
            # get current time as hour string format hh_dd-mm-yyyy
            time = datetime.datetime.now().strftime("%H_%d-%m-%Y")

            ts = datetime.datetime.now().strftime("%H-%M-%S")
            out = os.path.join(self.log_folder,os.path.basename("GIGA"), time, ts)
            if not os.path.exists(out):
                os.makedirs(out)

            for id, mesh in enumerate(meshes):
                mesh.export(os.path.join(out, f"mesh_inst_{id}.obj"))

            import trimesh
            from occupancy_prediction.utils.grasps import create_vgn_gripper_marker

            scenes = [trimesh.Scene() for _ in meshes]
            for id, grasp in enumerate(grasps_our_format):
                if grasp.score < 0.6:
                    continue

                gripper = create_vgn_gripper_marker(width = grasp.width, color = cmap(grasp.score)[:-1])
                se3_matrix = np.eye(4)
                se3_matrix[:3, -1] = grasp.position
                se3_matrix[:3, :3] = Rotation.from_quat(grasp.orientation).as_matrix()
                gripper_r = gripper.apply_transform(se3_matrix)
                # change color of gripper
                scenes[grasp.instance_id].add_geometry(gripper_r)

            for idx, scene in enumerate(scenes): 
                try:
                    scene.export(os.path.join(out, f"grasp_instance_{idx}.obj"))
                except:
                    pass
            o3d.io.write_point_cloud(os.path.join(out, f"cloud.ply"), pc)
            np.save(os.path.join(out, f"depth_img.npy"), img)
            import pickle
            pickle.dump({"loc": cam_loc, "ori": cam_orient}, open(os.path.join(out, f"cam_pose.pkl"), "wb"))
            print("Done logging meshes to ", out)


        def pub_best(grasp):
            self.current_grasp_marker.publish(get_grasp_marker(
                grasp.orientation, grasp.position, 0, color=np.array([0,0,1,1]), frame = self.base_frame
            ))


        if self.do_grasp:   
            if self.confirm_grasp:
                data_in = input("Press [g] to grasp, [r] to reset, [c] to continue...")
                if data_in == "r":
                    self.grasp_exe.reset()
                    self.reset_time = rospy.Time.now().to_sec()
                    return
                elif data_in == "g":
                    self.grasp_exe.pick_and_drop(grasps_our_format, cb = pub_best)
                    self.reset_time = rospy.Time.now().to_sec()
                return


        
            


if __name__ == "__main__":
    detector = RosGraspDetector()
    rospy.spin()
