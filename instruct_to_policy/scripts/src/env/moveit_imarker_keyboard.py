#!/usr/bin/env python
# coding: UTF-8
# Credic: https://blog.csdn.net/shenyan0712/article/details/102818106

import rospy
import sys, select, termios, tty
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerUpdate
import moveit_commander

def quat2rpy(quat):
    r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    return r.as_euler('xyz', degrees=False)

def rpy2quat(rpy):
    r = R.from_euler('xyz', rpy, degrees=False)
    return Quaternion(*r.as_quat())

class IMarkerKeyboard:
    def __init__(self):
        self.pre_pose= PoseStamped()
        self.planning_groups_tips = {}
        self.ttySettings=termios.tcgetattr(sys.stdin)
        # Get the current actual position of the robot
        group_name = "panda_arm"  # This is the joint group name defined for the robot in moveit configuration
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.robot = moveit_commander.RobotCommander()
        self.curPose=self.group.get_current_pose()
        self.curRPY=quat2rpy(self.curPose.pose.orientation)

        self.initState= self.robot.get_current_state()
        # Create related publishers and subscribers
        self.goalState_pub = rospy.Publisher("/rviz/moveit/update_custom_goal_state",
                                           RobotState,queue_size=5)
        self.marker_pub = rospy.Publisher("/rviz/moveit/move_marker/goal_panda_link8",
                                           PoseStamped,queue_size=5)
        self.update_sub = rospy.Subscriber("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update",
                                           InteractiveMarkerUpdate,self.updateGoal)
        # Ensure that rviz can receive messages
        rospy.sleep(2)  
        # Move the IMarker to the actual position
        self.goalState_pub.publish(self.initState)
        rospy.sleep(1)

    # Callback function updates the target position of the marker by receiving the update Topic
    def updateGoal(self, msg):
        if len(msg.poses) > 0:
            self.curPose.pose.position = msg.poses[0].pose.position
            self.curPose.pose.orientation = msg.poses[0].pose.orientation
            
            # update roll pitch yaw according to orientation
            self.curRPY=quat2rpy(self.curPose.pose.orientation)
            

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin],[],[],0)
        key= sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.ttySettings)
        print(key)
        return key


    def run(self):
        print("wait key...")
        while(True) :
            key = self.getKey()
            # Change the position of the marker
            if key=='-':
                goalPose=self.curPose
                goalPose.pose.position.z -=0.01
                self.marker_pub.publish(goalPose)
            elif key=='=':
                goalPose=self.curPose
                goalPose.pose.position.z +=0.01
                self.marker_pub.publish(goalPose)
            elif key=='a':
                goalPose=self.curPose
                goalPose.pose.position.x -=0.01
                self.marker_pub.publish(goalPose)
            elif key == 'd':
                goalPose =self.curPose
                goalPose.pose.position.x += 0.01
                self.marker_pub.publish(goalPose)
            elif key == 's':
                goalPose =self.curPose
                goalPose.pose.position.y -= 0.01
                self.marker_pub.publish(goalPose)
            elif key == 'w':
                goalPose =self.curPose
                goalPose.pose.position.y += 0.01
                self.marker_pub.publish(goalPose)
            # Change the orientation of the marker
            elif key == 'u': # roll positive
                roll = self.curRPY[0] + 0.01
                # update orientation according to roll
                goalPose = self.curPose
                goalPose.pose.orientation = rpy2quat([roll, self.curRPY[1], self.curRPY[2]])
                self.marker_pub.publish(goalPose)
            elif key == 'i': # roll negative
                roll = self.curRPY[0] - 0.01
                # update orientation according to roll
                goalPose = self.curPose
                goalPose.pose.orientation = rpy2quat([roll, self.curRPY[1], self.curRPY[2]])
                self.marker_pub.publish(goalPose)
            elif key == 'j': # pitch positive   
                pitch = self.curRPY[1] + 0.01
                # update orientation according to pitch
                goalPose = self.curPose
                goalPose.pose.orientation = rpy2quat([self.curRPY[0], pitch, self.curRPY[2]])
                self.marker_pub.publish(goalPose)
            elif key == 'k': # pitch negative
                pitch = self.curRPY[1] - 0.01
                # update orientation according to pitch
                goalPose = self.curPose
                goalPose.pose.orientation = rpy2quat([self.curRPY[0], pitch, self.curRPY[2]])
                self.marker_pub.publish(goalPose)
            elif key == 'n': # yaw positive
                yaw = self.curRPY[2] + 0.01
                # update orientation according to yaw
                goalPose = self.curPose
                goalPose.pose.orientation = rpy2quat([self.curRPY[0], self.curRPY[1], yaw])
                self.marker_pub.publish(goalPose)
            elif key == 'm': # yaw negative
                yaw = self.curRPY[2] - 0.01
                # update orientation according to yaw
                goalPose = self.curPose
                goalPose.pose.orientation = rpy2quat([self.curRPY[0], self.curRPY[1], yaw])
                self.marker_pub.publish(goalPose)
            if(key == '\x03'):  # Ctrl+C
                break

def main():
    rospy.init_node('marker_test', anonymous=True)
    app=IMarkerKeyboard()
    app.run()
    return

if __name__ == '__main__':
    main()
