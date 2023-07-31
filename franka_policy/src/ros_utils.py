#!/usr/bin/env python3

import rospy
import geometry_msgs.msg

from numpy.typing import NDArray
import rospy
from sensor_msgs.msg import PointCloud2

import open3d as o3d
from visualization_msgs.msg import Marker



from open3d_ros_helper import open3d_ros_helper as orh

def get_stamped_pose(position, orientation, frame):
    stamped_pose = geometry_msgs.msg.PoseStamped()
    stamped_pose.header.frame_id = frame
    stamped_pose.header.stamp = rospy.Time.now()
    stamped_pose.pose.position.x = position[0]
    stamped_pose.pose.position.y = position[1]
    stamped_pose.pose.position.z = position[2]
    stamped_pose.pose.orientation.x = orientation[0]
    stamped_pose.pose.orientation.y = orientation[1]
    stamped_pose.pose.orientation.z = orientation[2]
    stamped_pose.pose.orientation.w = orientation[3]
    return stamped_pose

def get_pose_msg(position, orientation):
    pose_goal = geometry_msgs.msg.Pose() 
    pose_goal.orientation.x = orientation[0]
    pose_goal.orientation.y = orientation[1]
    pose_goal.orientation.z = orientation[2]
    pose_goal.orientation.w = orientation[3]

    pose_goal.position.x = position [0]
    pose_goal.position.y = position[1]
    pose_goal.position.z = position[2]
    return pose_goal


def get_cloud(pts:NDArray , classes_colors:NDArray , frame = "world"):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(pts)
    cloud.colors = o3d.utility.Vector3dVector(classes_colors)
    rosc: PointCloud2 = orh.o3dpc_to_rospc(cloud)
    rosc.header.frame_id = frame
    return rosc

def get_grasp_marker(
    orientation,
    position,
    id,
    color=None,
    mesh_resource="file:///home/zrene/gripper_base_mesh.obj",
    ns="",
    frame="world",
):
    marker = Marker()

    marker.header.frame_id = frame
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