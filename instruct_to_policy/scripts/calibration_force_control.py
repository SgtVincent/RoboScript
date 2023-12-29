import rospy
from franka_msgs.srv import SetLoad

# 假设相机质量为0.5kg，质心相对于末端执行器的坐标为[0.1, 0, 0]
rospy.wait_for_service('/franka_control/set_load')
set_load = rospy.ServiceProxy('/franka_control/set_load', SetLoad)
print("!")
set_load(0.6, [0.0656815,-4.04594e-05,0.0562178], [0.00012084, 0, 0, 0, 0.00031538, 0, 0, 0, 0.00046712]) 
