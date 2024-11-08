#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Empty, EmptyResponse

class MoveToSpecificPoint:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_to_specific_point', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")

        self.point_sub = rospy.Subscriber('/detected_object_point', PointStamped, self.point_callback)
        self.target_point = None

        self.movement_complete_service = rospy.Service('movement_complete', Empty, self.handle_movement_complete)
        rospy.loginfo("MoveToSpecificPoint node initialized")

    def handle_movement_complete(self, req):
        rospy.loginfo("Movement complete service called")
        return EmptyResponse()

    def point_callback(self, msg):
        rospy.loginfo(f"Received point: {msg.point.x}, {msg.point.y}, {msg.point.z}")
        self.target_point = msg.point
        self.move_to_point(self.target_point.x, self.target_point.y, 0.240 - self.target_point.z)
        rospy.wait_for_service('movement_complete')

    def move_to_point(self, x, y, z):
        rospy.loginfo(f"Moving to (x, y, z): ({x}, {y}, {z})")
        current_pose = self.arm_group.get_current_pose().pose

        # Step 1: Move to x, y keeping the current z
        pose_goal_xy = geometry_msgs.msg.Pose()
        pose_goal_xy.orientation = current_pose.orientation
        pose_goal_xy.position.x = x
        pose_goal_xy.position.y = y
        pose_goal_xy.position.z = current_pose.position.z

        self.arm_group.set_pose_target(pose_goal_xy)
        plan_xy = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        if not plan_xy:
            rospy.logerr("Failed to plan path to the target x, y coordinates")
            return

        rospy.loginfo("Successfully moved to target x, y coordinates")

        # Step 2: Move to the target z keeping x, y
        pose_goal_z = geometry_msgs.msg.Pose()
        pose_goal_z.orientation = current_pose.orientation
        pose_goal_z.position.x = x
        pose_goal_z.position.y = y
        pose_goal_z.position.z = z

        self.arm_group.set_pose_target(pose_goal_z)
        plan_z = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        if not plan_z:
            rospy.logerr("Failed to plan path to the target z coordinate")
        else:
            rospy.loginfo("Successfully moved to target z coordinate")
            rospy.ServiceProxy('movement_complete', Empty)()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        mover = MoveToSpecificPoint()
        mover.run()
    except rospy.ROSInterruptException:
        pass

