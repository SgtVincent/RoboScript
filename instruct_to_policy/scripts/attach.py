import rospy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import moveit_commander
import sys

# 初始化ROS节点
rospy.init_node('print_acm_node')
moveit_commander.roscpp_initialize(sys.argv)
scene = moveit_commander.PlanningSceneInterface()

# 等待服务变得可用
rospy.wait_for_service('/get_planning_scene')

# 创建服务的代理，以便调用它
get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)

# 请求AllowedCollisionMatrix
request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
response = get_planning_scene(request)

# 打印AllowedCollisionMatrix
acm = response.scene.allowed_collision_matrix

# 打印ACM的所有条目
for i in range(len(acm.entry_names)):
    for j in range(len(acm.entry_names)):
        # 只有当有特定的允许碰撞设置时才打印信息
        if i < len(acm.entry_values) and j < len(acm.entry_values[i].enabled):
            if acm.entry_values[i].enabled[j]:
                print("Collision between '{}' and '{}' is allowed".format(acm.entry_names[i], acm.entry_names[j]))
