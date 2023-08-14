from typing import List
from geometry_msgs.msg import Pose, PoseStamped


################## ROS utils ###################

def get_pose_msg(position: List, orientation: List) -> Pose:
    pose_msg = Pose()
    pose_msg.position.x = position[0]
    pose_msg.position.y = position[1]
    pose_msg.position.z = position[2]
    pose_msg.orientation.x = orientation[0]
    pose_msg.orientation.y = orientation[1]
    pose_msg.orientation.z = orientation[2]
    pose_msg.orientation.w = orientation[3]
    return pose_msg


def get_stamped_pose(position: List, orientation: List, frame_id="world") -> PoseStamped:
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.pose = get_pose_msg(position, orientation)
    return pose_stamped