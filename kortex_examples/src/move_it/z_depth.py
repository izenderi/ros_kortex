#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class MoveInZ:
    def __init__(self):
        # Initialize the MoveIt commander and a ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_in_z', anonymous=True)

        # Initialize the robot, scene, and group commander
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")

        # Publisher to visualize the planned trajectory in RViz
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

    def move_in_z(self, z_distance):
        # Get the current pose of the end-effector
        current_pose = self.arm_group.get_current_pose().pose

        # Create a new pose goal
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = current_pose.orientation  # Keep the current orientation
        pose_goal.position.x = current_pose.position.x
        pose_goal.position.y = current_pose.position.y
        pose_goal.position.z = current_pose.position.z + z_distance  # Move in the z direction

        # Set the pose target
        self.arm_group.set_pose_target(pose_goal)

        # Plan the trajectory to the goal
        plan = self.arm_group.plan()
        planned_trajectory = plan[1]

        # Visualize the plan in RViz
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(planned_trajectory)
        self.display_trajectory_publisher.publish(display_trajectory)

        # Execute the motion
        success = self.arm_group.execute(planned_trajectory, wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        # Check if the planning was successful
        if not success:
            rospy.logerr("Failed to plan path to the target point")
        else:
            rospy.loginfo("Successfully planned and executed move to target point")

    def run(self):
        # Move downward by a specified distance (e.g., -0.1 meters)
        self.move_in_z(-0.085)

if __name__ == '__main__':
    try:
        mover = MoveInZ()
        mover.run()
    except rospy.ROSInterruptException:
        pass

