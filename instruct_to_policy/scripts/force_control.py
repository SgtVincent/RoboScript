import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
import sys
import copy

# 力阈值
FORCE_THRESHOLD = 6

def franka_state_callback(data):
    x_force = data.wrench.force.x
    y_force = data.wrench.force.y
    z_force = data.wrench.force.z
    # rospy.loginfo("Current axis force,%f,%f,%f", x_force,y_force,z_force)
    global current_force
    current_force = x_force
    # print(data)


# 初始化变量
current_force = 0

# 初始化 ROS 节点
rospy.init_node('move_franka', anonymous=True)
rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, franka_state_callback)

# 初始化 MoveIt!
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# 获取当前位姿
current_pose = move_group.get_current_pose().pose
initial_force = current_force
while initial_force == 0:
    initial_force = current_force
    print (initial_force)
    import time
    time.sleep(0.1)
# 设置笛卡尔路径点
waypoints = []
wpose = geometry_msgs.msg.Pose()
wpose.orientation = current_pose.orientation
wpose.position.x = current_pose.position.x + 0.3  # 直线移动 0.3 米
wpose.position.y = current_pose.position.y
wpose.position.z = current_pose.position.z
waypoints.append(copy.deepcopy(wpose))

# 计算笛卡尔路径
(plan, fraction) = move_group.compute_cartesian_path(
                             waypoints,   # 路径点
                             0.0001,        # 步进值
                             0.0,avoid_collisions = True)         # 跳跃阈值


# 执行计划
if fraction>0.2:
    move_group.execute(plan, wait=False)
    print("--------------------------begin--------------------------")
else:
    rospy.logwarn(f"MoveitEnv: Could not plan cartesian_path to target pose \n{wpose}.\n Plan accuracy: {fraction}")

# 循环检查力阈值和位置
while not rospy.is_shutdown():
    print(current_force-initial_force)
    if current_force - initial_force > FORCE_THRESHOLD:
        print("检测到力超过阈值")
        break
    if move_group.get_current_pose().pose.position.x >= wpose.position.x:
        print("达到目标位置")
        break
    if current_force == 0:
        break
# 清除目标并关闭 MoveIt
move_group.stop()
move_group.clear_pose_targets()
# moveit_commander.roscpp_shutdown()
# rospy.signal_shutdown("任务完成")
