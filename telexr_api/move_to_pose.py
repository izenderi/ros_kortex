#!/usr/bin/env python3

import sys
import rospy
import time
import math
from threading import Thread
from kortex_driver.srv import *
from kortex_driver.msg import *

class ExampleCartesianActionsWithNotifications:
    def __init__(self):
        try:
            rospy.init_node('example_cartesian_poses_with_notifications_python')

            self.HOME_ACTION_IDENTIFIER = 2
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            rospy.loginfo("Using robot_name " + self.robot_name)

            # Initialize attributes for buffered waypoints
            self.buffered_way_points = []
            self.last_action_notif_type = None
            self.all_notifs_succeeded = True

            # Initialize service proxies and subscribers
            self.action_topic_sub = rospy.Subscriber(
                "/" + self.robot_name + "/action_topic",
                ActionNotification,
                self.cb_action_topic
            )

            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

            self.is_init_success = True
        except:
            self.is_init_success = False

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if self.last_action_notif_type == ActionEvent.ACTION_END:
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif self.last_action_notif_type == ActionEvent.ACTION_ABORT:
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        else:
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED
        try:
            self.set_cartesian_reference_frame(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            rospy.sleep(0.25)
            return True

    def example_subscribe_to_a_robot_notification(self):
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")
            rospy.sleep(1.0)
            return True

    def buffer_pose(self, period=0.1):
        while not rospy.is_shutdown():
            try:
                with open('data', 'r') as file:
                    data = file.read().strip()
                with open('t1', 'r') as file:
                    time_end_bridge = file.read().strip()
            except FileNotFoundError:
                rospy.logwarn("Data or t1 file not found. Waiting...")
                time.sleep(period)
                continue

            if data:
                try:
                    time1, data_string = data.split(":", 1)
                    six_dof = eval(data_string.strip())  # Use ast.literal_eval if possible for safety
                except (ValueError, SyntaxError):
                    rospy.logwarn("Invalid data format. Skipping...")
                    continue

                x = 0.5 + six_dof[2] * 1
                y = 0.0 + six_dof[0] * -1
                z = 0.5 + six_dof[1] * 1

                if [x, y, z] not in self.buffered_way_points:
                    self.buffered_way_points.append([x, y, z])

            time.sleep(period)

    def move_to_buffered_poses(self):
        while not rospy.is_shutdown():
            if len(self.buffered_way_points) == 0:
                continue

            goal = ExecuteActionRequest()
            goal.input.name = "buffered_waypoints_sequence"
            goal.input.handle.action_type = ActionType.REACH_POSE
            goal.input.handle.identifier = 1000

            # Prepare and send pose 1
            # my_cartesian_speed = CartesianSpeed()
            # my_cartesian_speed.translation = 2.5 # m/s
            # my_cartesian_speed.orientation = 150  # deg/s

            for waypoint in self.buffered_way_points:
                constrained_pose = ConstrainedPose()

                # constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

                constrained_pose.target_pose.x = waypoint[0]
                constrained_pose.target_pose.y = waypoint[1]
                constrained_pose.target_pose.z = waypoint[2]
                constrained_pose.target_pose.theta_x = 90
                constrained_pose.target_pose.theta_y = 0
                constrained_pose.target_pose.theta_z = 90

                goal.input.oneof_action_parameters.reach_pose.append(constrained_pose)

            rospy.loginfo(f"Sending {len(self.buffered_way_points)} waypoints...")
            self.last_action_notif_type = None

            try:
                self.execute_action(goal)
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to send waypoints: {e}")
                continue
            else:
                self.wait_for_action_end_or_abort()

            self.buffered_way_points.clear()  # Clear buffered waypoints after sending

    def main(self):
        success = self.is_init_success

        if success:
            success &= self.example_clear_faults()
            success &= self.example_home_the_robot()
            success &= self.example_set_cartesian_reference_frame()

            buffer_thread = Thread(target=self.buffer_pose)
            move_thread = Thread(target=self.move_to_buffered_poses)

            buffer_thread.start()
            move_thread.start()

            buffer_thread.join()
            move_thread.join()

        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = ExampleCartesianActionsWithNotifications()
    ex.main()
