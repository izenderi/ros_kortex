#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

class MoveToSpecificPoint:
    def __init__(self):
        # Initialize the MoveIt commander and a ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_to_specific_point', anonymous=True)

        # Initialize the robot, scene, and group commander
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")

    def move_to_point(self, x, y):
        # Get the current pose of the end-effector
        current_pose = self.arm_group.get_current_pose().pose

        # Create a new pose goal
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = current_pose.orientation  # Keep the current orientation
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = current_pose.position.z  # Keep the current z position

        # Set the pose target
        self.arm_group.set_pose_target(pose_goal)

        # Plan and execute the motion
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        # Check if the planning was successful
        if not plan:
            rospy.logerr("Failed to plan path to the target point")
        else:
            rospy.loginfo("Successfully planned and executed move to target point")

    def run(self):
        # Move to the specific point (0.40, -0.04)
        self.move_to_point(0.50, 0.04)

if __name__ == '__main__':
    try:
        mover = MoveToSpecificPoint()
        mover.run()
    except rospy.ROSInterruptException:
        pass

