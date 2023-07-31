#!/usr/bin/env python3

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

sys.path.insert(0, "/home/zrene/git/LatentQueryRefinement/")
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

WITH_RECONSTRUCTION = False

class GraspExecutor:

    def computeIK(self, orientation, position):
        # Create a pose to compute IK for
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.frame # Hard coded for now
        pose_stamped.header.stamp = rospy.Time.now()

        pose_stamped.pose.orientation.x = orientation[0]
        pose_stamped.pose.orientation.y = orientation[1]
        pose_stamped.pose.orientation.z = orientation[2]
        pose_stamped.pose.orientation.w = orientation[3]

        pose_stamped.pose.position.x = position [0]
        pose_stamped.pose.position.y = position[1]
        pose_stamped.pose.position.z = position[2]

        # Create a moveit ik request
        ik_request = PositionIKRequest() 
        ik_request.group_name = 'panda_manipulator' # Hard coded for now
        ik_request.ik_link_name="panda_hand"
        ik_request.pose_stamped = pose_stamped
        #ik_request.timeout.secs = 0.1
        ik_request.avoid_collisions = True 
        
        request_value = self.compute_ik(ik_request)
        if request_value.error_code.val == -31:
            print("NO IK SOLUTION FOUND")
        if request_value.error_code.val == 1:
            joint_positions = request_value.solution.joint_state.position[1:7] 
            joint_names = request_value.solution.joint_state.name[1:7]
            print("FOUND IK")
            return joint_positions,joint_names
        else:
            return None,None


    def load_scene(self):
        wall_pose = geometry_msgs.msg.PoseStamped()
        wall_pose.header.frame_id = "panda_link0"
        wall_pose.pose.orientation.w = 1.0
        wall_pose.pose.orientation.x = 0.0
        wall_pose.pose.orientation.y = 0.0
        wall_pose.pose.orientation.z = 0.0

        wall_pose.pose.position.x = 0
        wall_pose.pose.position.y = 0.47
        wall_pose.pose.position.z = 0.0
        self.scene.add_box("wall", wall_pose, size=(3, 0.02, 3))
        # Misc variables
        wall_pose = geometry_msgs.msg.PoseStamped()
        wall_pose.header.frame_id = "panda_link0"
        wall_pose.pose.orientation.w = 1.0
        wall_pose.pose.orientation.x = 0.0
        wall_pose.pose.orientation.y = 0.0
        wall_pose.pose.orientation.z = 0.0

        wall_pose.pose.position.x = 1.7
        wall_pose.pose.position.y = 0.0
        wall_pose.pose.position.z = -0.015
        self.scene.add_box("floor", wall_pose, size=(3, 3, 0))

    def __init__(self, frame) -> None:
        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        self.compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        robot = moveit_commander.RobotCommander()
        self.frame = frame

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        self.scene = moveit_commander.PlanningSceneInterface(service_timeout=10)
        group_name = "panda_manipulator"
        group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers = 15)
        ## We create a `DisplayTrajectory`_ publisher which is used later to publish
        ## trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        self.load_scene()
        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.group = group
        self.moving = False
        self.gripper = moveit_commander.MoveGroupCommander("panda_hand")


    # def drop_off(self, joints = [-1.2418474324399773, -0.04365144067230651, -0.050630739720749725, -2.013370735752409, -0.08598059593452781, 1.978366626257196, -0.4705251475721925]):
    def drop_off(self, joints = [    -0.6861938888210998, -1.0922074558700297, -0.596633734874051, -2.397880921082069, -0.5792871115412288, 1.3971697680950166, -1.9250296761749517]):
    
        group = self.group
        group.set_goal_tolerance(0.005)
        group.set_planner_id("RRTConnect")
        group.set_max_velocity_scaling_factor(0.3)
        group.set_max_acceleration_scaling_factor(0.3)
        # pose_goal.position.x = 0.4
        group.set_joint_value_target(joints)
        
        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # print("moved?")
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

    def open_gripper(self):
        group = self.gripper

        # pose_goal.position.x = 0.4
        group.set_joint_value_target([0.039, 0.039])
        
        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # print("moved?")
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()
        time.sleep(2)
        print("should have opened!")


    def grasp(self, width):
        self.moving = True
        # Creates the SimpleActionClient, passing the type of the action
        # (GraspAction) to the constructor.
        client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = franka_gripper.msg.GraspGoal()
        goal.width = width
        goal.epsilon.inner = 0.04
        goal.epsilon.outer = 0.04
        goal.speed = 0.15
        goal.force = 20

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()
        self.moving = False

        # Prints out the result of executing the action
    def reset(self):
        self.moving = True
        for obj in self.scene.get_known_object_names():
            if "inst" in obj:
                print("removing ", obj)
                self.scene.remove_world_object(obj)
        reset_pos = [-0.08251470748374336, 0.19599197531792156, 0.841849711995378, -1.7190730213700678, -0.6383234506829698, 1.3664373589356658, 0.030431373252636857]
        # reset_pos = [-0.0745181431487993, 0.4494850737236856, 0.8270193549708322, -1.5205520947895879, -0.8831002754171688, 1.3287292592525481, 0.00643488985824300]
        # reset_pos = [-0.5861821080702139, 0.055900042107092915, -0.3281157743728023, -1.826453942497152, 0.7512797036630254, 1.6094823167995767, 2.091976654118447]
        #reset_pos = [-0.5878410830476827, -0.28344825294979825, 1.1282720037594176, -1.874332442399111, -0.31171058199803037, 1.8769036887578303, -0.5659639980552924]
        group = self.group
        group.set_goal_tolerance(0.01)
        group.set_planner_id("RRTConnect")
        group.set_max_velocity_scaling_factor(0.4)
        group.set_max_acceleration_scaling_factor(0.4)
        # pose_goal.position.x = 0.4
        group.set_joint_value_target(reset_pos)
        #self.go_to_pose_goal([-8.66025404e-01, -2.65143810e-17,  1.53080850e-17, -5.00000000e-01], [0.15 + 35     , -0.16160254,  0.47009619])
        #return
        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # print("moved?")
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()
        # sleeping until read
        print("sleeping from reset")
        time.sleep(2)
        self.moving = False


    def go_to_pose_goal(self, orientation, position):
        self.moving = True
        t1 = time.time()
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group
        self.group.set_pose_reference_frame(self.frame)
        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()        # print("current_pose", current_pose)
        pose_goal.orientation.x = orientation[0]
        pose_goal.orientation.y = orientation[1]
        pose_goal.orientation.z = orientation[2]
        pose_goal.orientation.w = orientation[3]

        pose_goal.position.x = position [0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]
        print(pose_goal)
        group.set_goal_tolerance(0.01)
        group.set_planner_id("RRTConnect")

        group.set_max_velocity_scaling_factor(0.2)
        group.set_max_acceleration_scaling_factor(0.2)

        # pose_goal.position.x = 0.4
        group.set_pose_target(pose_goal)
        
        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # print("moved?")
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()
        self.moving = False
        return plan


class RosGraspDetector:
    def __init__(self) -> None:

        rospy.init_node("grasp_detector")
        # General Config
        self.base_frame = "workspace"
        self.sensor_frame = "camera_color_optical_frame"
        self.center_pc = True


        # TODO, set to value
        # self.do_grasp = False
        self.do_grasp = True
        self.use_giga = True


        self.rate = 0.4    # Hz

        self.grasp_exe = GraspExecutor(self.base_frame)
        self.grasp_exe.open_gripper()
        self.grasp_exe.moving = False
        # self.grasp_exe.go_to_pose_goal([-0.456, 0.807, 0.236, -0.292], [0.316, 0.568, 0.514])
        # , 0.00010309666686225682, 0.00010309666686225682]
        self.grasp_exe.reset()
        self.last_ts = 0

        self.tf_listener = tf.TransformListener()

        if not self.use_giga:
            from model_wrapper import ModelWrapper

            self.model = ModelWrapper(config="contact_packed_width.yaml")
            self.pc_sub = rospy.Subscriber(
                        "/cropbox/output", PointCloud2, self.pc_callback, queue_size=1
                    )
        else:
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



        self.n_grasps = 10
        self.grasp_marker_pub = rospy.Publisher(
            "/grasps", MarkerArray, queue_size=self.n_grasps
        )
        self.pc_pub = rospy.Publisher("/segmented_pc", PointCloud2, queue_size=1)
        self.mesh_marker_pub = rospy.Publisher("/mesh", MarkerArray, queue_size=10)
        self.cam_loc_pub = rospy.Publisher("/cam_loc", Marker, queue_size=10)
        print("********** Grasp Detector Ready!")


    def get_cloud(self, pts, classes_colors):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pts)
        cloud.colors = o3d.utility.Vector3dVector(classes_colors)

        rosc: PointCloud2 = orh.o3dpc_to_rospc(cloud)
        rosc.header.frame_id = self.base_frame
        return rosc

    def get_grasp_marker(
        self,
        orientation,
        position,
        id,
        color=None,
        mesh_resource="file:///home/zrene/gripper_base_mesh.obj",
        ns="",
    ):
        marker = Marker()

        marker.header.frame_id = self.base_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns

        # Shape (mesh resource type - 10)
        marker.type = 10
        marker.id = id
        marker.action = 0

        # Note: Must set mesh_resource to a valid URL for a model to appear
        if mesh_resource is not None:
            marker.mesh_resource = mesh_resource

        # Scale
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Color
        if color is not None:
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

        marker.color.a = 1.0

        # Pose
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        return marker

    def img_callback(self, img_msg: Image):
        if (img_msg.header.stamp.to_sec() - self.last_ts) < 1 / self.rate:
            print("skipping")
            return
        self.last_ts = img_msg.header.stamp.to_sec()
        
        print("got img")

        img = self.cv_bridge.imgmsg_to_cv2(img_msg).astype(np.float32) * 0.001
        print("Converted")
        self.tf_listener.waitForTransform(
            self.base_frame, self.sensor_frame, rospy.Time(0), rospy.Duration(10.0)
        )
        cam_loc, cam_orient = self.tf_listener.lookupTransform(self.sensor_frame, self.base_frame, rospy.Duration(0))
        tsdf_vol = TSDFVolume(0.3, 40)
        cam_intrinsics = CameraIntrinsic.from_dict({
            "height": 480,
            "width": 640,
            "K":  [381.28717041015625, 0.0, 317.65496826171875, 0.0, 381.28717041015625, 240.4567413330078, 0.0, 0.0, 1.0]

        })
        cam_tf = Transform(Rotation.from_quat(cam_orient), cam_loc)

        tsdf_vol.integrate(img, cam_intrinsics, cam_tf) 
        pc = tsdf_vol.get_cloud()
        pred_mesh, _ = self.generator.generate_mesh({'inputs': torch.from_numpy(tsdf_vol.get_grid()).cuda()})
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

        # extrinsic = extrinsic.as_matrix()
        # import pdb; pdb.set_trace()
        # o3d.visualization.draw(o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic, extrinsic))
        # import pdb; pdb.set_trace()

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

            marker = self.get_grasp_marker(
                rot, c, id, color=cmap(1*(scores[id] - 0.4) + 0.4 )  if s != np.max(scores) else np.array([0,0,1,1])
            )

            if id == 0:
                marker.action = Marker.DELETEALL
            arr.markers.append(marker)
        self.grasp_marker_pub.publish(arr)


        # publishes meshes. currently done ugly!
        arr = MarkerArray()
        for id, mesh in enumerate(meshes):
            f = "/tmp/mesh_inst_{}_{}.obj".format(id,self.last_ts)
            mesh.export(f)
            print(f)
            marker = self.get_grasp_marker(
                [0, 0, 0, 1],
                np.asarray([0, 0, 0]),
                id + len(ori) + 1,
                color=inst_cmap(id / 10),
                mesh_resource="file://" + f,
                ns="obj_" + str(id),
            )
            if id == 0:
                marker.action = Marker.DELETEALL
            arr.markers.append(marker)

            p = geometry_msgs.msg.PoseStamped()
            p.header.frame_id = self.base_frame
            p.pose.orientation.w = 1.0
            p.pose.position.x = 0
            p.pose.position.y = 0
            p.pose.position.z = 0

            self.grasp_exe.scene.add_mesh(f"inst_{id}", p, f, size = (1, 1, 1))

        self.mesh_marker_pub.publish(arr)



    def pc_callback(self, pointcloud: PointCloud2):
        if (pointcloud.header.stamp.to_sec() - self.last_ts) < 1 / self.rate:
            print("skipping")
            return
        
        if self.grasp_exe.moving:
            print("Gras executor moving. skipping")
            return

        try:
            for obj in self.grasp_exe.scene.get_known_object_names():
                if "inst" in obj:
                    print("removing ", obj)
                    self.grasp_exe.scene.remove_world_object(obj)


            self.last_ts = pointcloud.header.stamp.to_sec()

            # Transform the pointcloud to the base frame
            self.tf_listener.waitForTransform(
                self.base_frame,   self.sensor_frame,rospy.Time(0), rospy.Duration(10.0)
            )
            cam_loc, _ = self.tf_listener.lookupTransform(
                self.base_frame, self.sensor_frame, rospy.Duration(0)
            )
            self.tf_listener.waitForTransform(
                "world",   self.base_frame, rospy.Time(0), rospy.Duration(10.0)
            )
            base_frame, _ = self.tf_listener.lookupTransform(
                "world", self.base_frame, rospy.Duration(0)
            )


            print("cam_loc", cam_loc)
            m = self.get_grasp_marker(
                    [0, 0, 0, 1],
                    cam_loc,
                    2,
                    mesh_resource=None)
            m.type = 2
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.scale.z = 0.05
            self.cam_loc_pub.publish(m)




            points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pointcloud)
            center_pt = np.asarray([0, 0, 0])
            if self.center_pc:
                print("centering")
                center_pt = np.mean(points, axis=0)
                points = points - center_pt
                cam_loc = cam_loc - center_pt
            print(points.shape, cam_loc)

            pc = torch.from_numpy(points).float()
            colors = 0 * pc + 1

            o3dc = o3d.geometry.PointCloud()
            o3dc.points = o3d.utility.Vector3dVector(pc[..., :3].numpy())
            o3dc.colors = o3d.utility.Vector3dVector(colors.numpy() / 255)

            # o3dc = o3dc.voxel_down_sample(voxel_size=0.0005)
            # o3dc, _ = o3dc.remove_radius_outlier(nb_points=10, radius=0.005)
            pc = torch.from_numpy(np.asarray(o3dc.points)).float().cuda()
            o3dc.estimate_normals()
            if cam_loc is not None:
                o3dc.orient_normals_towards_camera_location(cam_loc)

            ds = o3dc.voxel_down_sample(voxel_size=0.008)

            grasp_pts = torch.from_numpy(np.asarray(ds.points)).float().cuda()
            grasp_normals = torch.from_numpy(np.asarray(ds.normals)).float().cuda()
            grasp_angle = grasp_normals[:, -1].abs() / grasp_normals.norm(dim=1)
            print("pc shape", pc.shape)
            for th in [ 0.75, 1, np.inf]:
                if (grasp_angle < th).sum() != 0:
                    grasp_pts = grasp_pts[grasp_angle < th]
                    grasp_normals = grasp_normals[grasp_angle < th]
                    break
            


            out = self.model(
                pc,
                normals=torch.from_numpy(np.asarray(o3dc.normals)).float(),
                grasp_pts=grasp_pts,
                grasp_normals=grasp_normals,
                visualize=False,
                n_grasps=self.n_grasps,
                each_object=True,
                return_meshes=WITH_RECONSTRUCTION,
                return_classes=True,
                return_latent_ids=True,
                return_instances=True,
                mesh_resolution = 32
            )
            if WITH_RECONSTRUCTION:
                ori, contact, scores, lat_id, meshes, (pts, classes), grasp_instances = out
            else:
                ori, contact, scores, lat_id, (pts, classes), grasp_instances = out
                meshes = []

            self.grasp_exe.moving = True
            print("pts shape", pts.shape)
            print("scores", np.min(scores), np.max(scores))
            valid = scores > 0.6 # threshold
            if not valid.any():
                print("no good grasps found.")
                valid[np.argmax(scores)] = True
            ori = ori[valid]
            contact = contact[valid]
            scores = scores[valid]
            grasp_instances = grasp_instances[valid]

            classes_colors = inst_cmap(classes / 10)[:, :3]

            self.pc_pub.publish(self.get_cloud(pts + center_pt, classes_colors))

            arr = MarkerArray()
            for id, (rot, c, s) in enumerate(zip(ori, contact, scores)):

                marker = self.get_grasp_marker(
                    rot, c + center_pt, id, color=cmap(1.5*(scores[id] - 0.4) + 0.5 )  if s != np.max(scores) else np.array([0,0,1,1])
                )

                if id == 0:
                    marker.action = Marker.DELETEALL
                arr.markers.append(marker)
            self.grasp_marker_pub.publish(arr)


            # publishes meshes. currently done ugly!
            arr = MarkerArray()
            for id, mesh in enumerate(meshes):
                f = "/tmp/mesh_inst_{}_{}.obj".format(id,self.last_ts)
                mesh.export(f)
                print(f)
                marker = self.get_grasp_marker(
                    [0, 0, 0, 1],
                    center_pt,
                    id + len(ori) + 1,
                    color=inst_cmap(id / 10),
                    mesh_resource="file://" + f,
                    ns="obj_" + str(id),
                )
                if id == 0:
                    marker.action = Marker.DELETEALL
                arr.markers.append(marker)

                p = geometry_msgs.msg.PoseStamped()
                p.header.frame_id = self.base_frame
                p.pose.orientation.w = 1.0
                p.pose.position.x = center_pt[0]
                p.pose.position.y = center_pt[1]
                p.pose.position.z = center_pt[2]

                self.grasp_exe.scene.add_mesh(f"inst_{id}", p, f, size = (1, 1, 1))

            self.mesh_marker_pub.publish(arr)

            if not self.do_grasp:
                self.grasp_exe.moving = False
                return
            
            data_in = input("Press Enter to Grasp...")
            
            if data_in == "r":
                print("Resetting")
                self.grasp_exe.moving = False
                return
            
            orientations = ori
            
            for _ in range(5):
                for _ in range(20): # 20 ik attempts
                    best_grasp = np.argmax(scores)
                    pos, ori = contact[best_grasp] + center_pt, orientations[best_grasp]
                    target_id = int(grasp_instances[best_grasp].item())
                    print("Grasping target", target_id)
                    target_object = "inst_{}".format(target_id,self.last_ts)
                    target_mesh =  "/tmp/mesh_inst_{}_{}.obj".format(target_id,self.last_ts)

                    from scipy.spatial.transform import Rotation
                    print("Going to pre-grasp pose.")
                    ik_pos = pos +  Rotation.from_quat(ori).as_matrix() @ np.array([0,0,0.05])
                    if self.grasp_exe.computeIK(ori,ik_pos)[0] is None:
                        print("Invalid ik. Trying next grasp.")
                        scores[best_grasp] = 0
                    else:
                        ik_pos = pos + Rotation.from_quat(ori).as_matrix() @ np.array([0,0,-0.04])
                        if self.grasp_exe.computeIK(ori,pos)[0] is None:
                            print("Invalid ik for second stage. Trying next grasp.")
                            scores[best_grasp] = 0
                        else:
                            break

                print("going to pre grasp pose")
                pos = pos + Rotation.from_quat(ori).as_matrix()  @ np.array([0,0,-0.04])
                if not self.grasp_exe.go_to_pose_goal(ori, pos):
                    scores[best_grasp] = 0
                    continue
                
                print("Going to grasp pose.")
                pos = pos + Rotation.from_quat(ori).as_matrix() @ np.array([0,0,0.09])
                self.grasp_exe.go_to_pose_goal(ori, pos)
                print("go grasping")
                if WITH_RECONSTRUCTION:
                    self.grasp_exe.scene.remove_world_object(target_object)
                    
                #time.sleep(1)
                self.grasp_exe.grasp(0.05)
                #time.sleep(1)
                
                grasping_group = "panda_hand"
                touch_links = self.grasp_exe.robot.get_link_names(group=grasping_group)
                
                if WITH_RECONSTRUCTION:
                    self.grasp_exe.scene.add_mesh(target_object, p, target_mesh, size = (1, 1, 1))
                    self.grasp_exe.scene.attach_mesh("panda_hand", target_object, touch_links=touch_links)

                # self.grasp_exe.scene.attach_mesh("panda_link8", target_object, p, target_mesh, size = (1, 1, 1))
                # print("Going to post-grasp pose.")
                # pos = pos + np.array([0,0,0.25])
                # self.grasp_exe.go_to_pose_goal(ori, pos)
                print("go to dropoff")
                self.grasp_exe.drop_off()
                self.grasp_exe.open_gripper()
                if WITH_RECONSTRUCTION:
                    self.grasp_exe.scene.remove_attached_object("panda_hand", name=target_object)

                break

            self.grasp_exe.reset()
            self.grasp_exe.moving = False
        finally:
            self.grasp_exe.moving = False


if __name__ == "__main__":
    detector = RosGraspDetector()
    rospy.spin()
