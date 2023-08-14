#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped

def publish_ground_truth_pose():
    rospy.init_node('ground_truth_pose_publisher')
    
    # Create a publisher for the ground truth pose
    pose_pub = rospy.Publisher('/ground_truth_pose', PoseStamped, queue_size=10)
    
    # Create a client for the GetModelState service
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    
    rate = rospy.Rate(10)  # Publish at 10 Hz
    
    while not rospy.is_shutdown():
        try:
            # Query the pose of your object
            response = get_model_state(model_name='stone')
            pose = response.pose
            
            # Create a PoseStamped message
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = 'world'  # Assuming 'world' is the Gazebo reference frame
            pose_stamped.pose = pose
            
            # Publish the ground truth pose
            pose_pub.publish(pose_stamped)
            
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_ground_truth_pose()
    except rospy.ROSInterruptException:
        pass