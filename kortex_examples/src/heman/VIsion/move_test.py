#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String

def moveit_python_script():
    # Initialize the moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_python_script', anonymous=True)

    # Initialize MoveIt Commander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"  # this should match the arm planning group name in your MoveIt! configuration
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Allow replanning to increase the odds of a solution
    move_group.allow_replanning(True)

    # Set a target pose for the end-effector
    target_pose = PoseStamped()
    target_pose.header.frame_id = robot.get_planning_frame()
    target_pose.pose.position.x = 0.5  # Specify your desired position here
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.6
    target_pose.pose.orientation.w = 1.0  # Specify your desired orientation here

    move_group.set_pose_target(target_pose)

    # Plan and execute the trajectory
    plan = move_group.go(wait=True)

    # Print the result
    if plan:
        rospy.loginfo("Successfully planned and executed the trajectory!")
    else:
        rospy.logerr("Failed to plan trajectory")

    # Shutdown MoveIt Commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        moveit_python_script()
    except rospy.ROSInterruptException:
        pass
