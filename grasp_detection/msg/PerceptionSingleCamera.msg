# This message defines the data of the raw data and perception results of a single cameras

# camera id 
string camera_id

# RGB image data
sensor_msgs/Image rgb_image
# RGB Camera info
sensor_msgs/CameraInfo rgb_camera_info
# RGB Camera pose in the world frame
geometry_msgs/Pose rgb_camera_pose

# Depth image data
sensor_msgs/Image depth_image
# Depth Camera info
sensor_msgs/CameraInfo depth_camera_info
# Depth Camera pose 
geometry_msgs/Pose depth_camera_pose

# 2D bounding boxes of the detected objects
BoundingBox2D[] detections