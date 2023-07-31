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
        self.sensor_frame = "camera_color_optical_frame"

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

        from occupancy_prediction.wrapper.model_wrapper import ModelWrapper
        self.config = "/scratch/escratch/saved/Res16UNet34C_last_4_base/config.yaml"
        self.current_grasp_marker = rospy.Publisher("/current_grasp", Marker, queue_size=10)

        # gripper_offset=0.014,  # 1cm additional offset
        # gripper_offset_perc=0.1

        print("Creating Model")
        self.model = ModelWrapper(config=self.config, mesh_resolution=64, postprocess = True, n_orientations=12,n_grasp_pred_orientations=3, n_grasps=64, gripper_offset=0.01, gripper_offset_perc=0.0)
        self.model.eval()
        self.grasp_marker_pub = rospy.Publisher(
            "/grasps", MarkerArray, queue_size=1
        )
        self.pc_pub = rospy.Publisher("/segmented_pc", PointCloud2, queue_size=1)
        self.mesh_marker_pub = rospy.Publisher("/mesh", MarkerArray, queue_size=10)
        self.cam_loc_pub = rospy.Publisher("/cam_loc", Marker, queue_size=10)

        self.pc_sub = rospy.Subscriber(
            "/cropbox/output", PointCloud2, self.pc_callback, queue_size=1
        )
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
    def pc_callback(self, pointcloud: PointCloud2):

        # check pointcloud delay
        print("PC delay: ", rospy.Time.now().to_sec() - pointcloud.header.stamp.to_sec())
        if (self.reset_time - pointcloud.header.stamp.to_sec()) > 0:
            print("PC delay too large. Skipping.")
            return
        # Transform the pointcloud to the base frame
        self.tf_listener.waitForTransform(
            self.base_frame, self.sensor_frame,rospy.Time(0), rospy.Duration(10.0)
        )
        cam_loc, _ = self.tf_listener.lookupTransform(
            self.base_frame, self.sensor_frame, rospy.Duration(0)
        )
        import open3d as o3d

        cloud = orh.rospc_to_o3dpc(pointcloud, remove_nans=True).remove_radius_outlier(nb_points=10, radius=0.005)[0].voxel_down_sample(voxel_size=0.0025)
        ccords = np.asarray(cloud.points).copy()
        cloud.points = o3d.utility.Vector3dVector(np.asarray(cloud.points) + np.array([0.003, 0.0,0.013]))

        # while True:
        #     #         x0.007
        #     # y0.01
        #     # z0.015
        #     # r0
        #     # x

        #     x = float(input("x"))
        #     # y = float(input("y"))
        #     # z = float(input("z"))
        #     # r = input("r")
        #     # if r == "r":
        #     #     return

        #     # cloud.points = o3d.utility.Vector3dVector(ccords + np.array([x, y, z]))

        #     self.pc_pub.publish(get_cloud(np.asarray(cloud.points), np.asarray(cloud.colors), frame= self.base_frame))
            
        cloud.estimate_normals()
        if cam_loc is not None:
            cloud.orient_normals_towards_camera_location(cam_loc)

        
        ds = cloud.remove_radius_outlier(nb_points=5, radius=0.005)[0].voxel_down_sample(voxel_size=0.005)

        predictions = self.model(
            coords = torch.from_numpy(np.asarray(cloud.points)).float().to(self.model.device),
            colors = None,
            normals = None,
            grasp_pts = torch.from_numpy(np.asarray(ds.points)).float().to(self.model.device),
            grasp_normals = torch.from_numpy(np.asarray(ds.normals)).float().to(self.model.device),
            return_meshes = True,
            return_scene_grasps = True,
            return_object_grasps = False,
        )
        classes_colors = inst_cmap(predictions.class_predictions.cpu().numpy() / 10)[:, :3]
        geoms = []
        for m in predictions.reconstructions:
            geoms.append(m.as_open3d)
        o3d.visualization.draw(geoms)

        # self.pc_pub.publish(get_cloud(predictions.embedding.voxelized_pc.cpu().numpy(), classes_colors, frame= self.base_frame))
        self.pc_pub.publish(get_cloud(np.asarray(cloud.points), np.asarray(cloud.colors), frame= self.base_frame))

        arr = MarkerArray()
        grasps = []
        for grasp_id, grasp in enumerate(predictions.scene_grasp_poses):
            if grasp.score < 0.5:
                continue
            print("grasp:", grasp.width, grasp.score)
            marker = get_grasp_marker(
                orientation=grasp.orientation,
                position=grasp.position,
                id=grasp_id,
                color = cmap(grasp.score),
                ns=f"scene_grasps_inst_{grasp.instance_id}",
                frame= self.base_frame)
            
            if grasp_id == 0:
                marker.action = marker.DELETEALL
            grasps.append(grasp)
            arr.markers.append(marker)
        self.grasp_marker_pub.publish(arr)


        meshes = MarkerArray()
        for id, mesh in enumerate(predictions.reconstructions):
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
            meshes.markers.append(marker)
            
            self.grasp_exe.register_object(mesh, id, [0,0,0])
        self.mesh_marker_pub.publish(meshes)
        
        
        import os;
        if self.log_meshes:
            import datetime
            # get current time as hour string format hh_dd-mm-yyyy
            time = datetime.datetime.now().strftime("%H_%d-%m-%Y")

            ts = datetime.datetime.now().strftime("%H-%M-%S")
            out = os.path.join(self.log_folder,os.path.basename(os.path.dirname(self.config)), time, ts)
            if not os.path.exists(out):
                os.makedirs(out)

            for id, mesh in enumerate(predictions.reconstructions):
                mesh.export(os.path.join(out, f"mesh_inst_{id}.obj"))

            import trimesh
            from occupancy_prediction.utils.grasps import create_vgn_gripper_marker
            scenes = [trimesh.Scene() for _ in predictions.reconstructions]
            for id, grasp in enumerate(predictions.scene_grasp_poses):
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
            o3d.io.write_point_cloud(os.path.join(out, f"cloud.ply"), cloud)
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
                    self.grasp_exe.pick_and_drop(grasps, cb = pub_best)
                    self.reset_time = rospy.Time.now().to_sec()
                return

        return 
        
        if not self.grasp_exe.grasp(pos +  Rotation.from_quat(ori).as_matrix() @ np.array([0,0,0.05]), ori, object_id=target_id):
            print("Grasp failed. Trying next grasp.")
            


if __name__ == "__main__":
    detector = RosGraspDetector()
    rospy.spin()
