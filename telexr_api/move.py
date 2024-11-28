#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2021 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
import math

from threading import Thread

import actionlib

from kortex_driver.srv import *
from kortex_driver.msg import *

class WaypointActionClient:
    def __init__(self):
        try:
            rospy.init_node('example_waypoint_action_python')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            # Publisher for velocity commands
            self.velocity_publisher = rospy.Publisher(
                '/my_gen3/in/cartesian_velocity',
                TwistCommand,
                queue_size=10
            )

            # Initialize attributes for buffered waypoints
            self.buffered_way_points = []
            self.buffered_msg_ids = []
            self.last_action_notif_type = None
            self.all_notifs_succeeded = True

            # outut csv
            self.data_log = []

        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
       
        return cartesianWaypoint
    
    def FillTwistCommand(self, reference_frame, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, duration):
        # Create and publish the velocity command
        twist_command = TwistCommand()
        twist_command.reference_frame = reference_frame  # Base reference frame
        twist_command.twist.linear_x = linear_x
        twist_command.twist.linear_y = linear_y
        twist_command.twist.linear_z = linear_z
        twist_command.twist.angular_x = angular_x
        twist_command.twist.angular_y = angular_y
        twist_command.twist.angular_z = angular_z
        twist_command.duration = duration  # Continuous execution
        return twist_command
        

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
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
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
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
    
    def buffer_pose(self, period=0.05):
        while not rospy.is_shutdown():
            try:
                with open('data', 'r') as file:
                    data = file.read().strip()
                with open('t1', 'r') as file:
                    time_end_bridge = file.read().strip()
                with open('msg_id', 'r') as file:
                    msg_id = file.read().strip()
            except FileNotFoundError:
                rospy.logwarn("Data, t1, msg_id file not found. Waiting...")
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

                if msg_id not in self.buffered_msg_ids:
                    self.buffered_msg_ids.append(msg_id)
                    self.buffered_way_points.append([x, y, z])

                # rospy.loginfo(f"{len(self.buffered_way_points)} waypoints...")

            time.sleep(period)
    
    def publish_velocity(self):
        rate = rospy.Rate(100)  # 20Hz = 50ms sleep
        speed_factor = 1  # Scaling factor for the speed
        threshold = 0.01  # Threshold to consider the target reached

        while not rospy.is_shutdown():
            if not self.buffered_way_points:
                rospy.loginfo("No waypoints in buffer. Stopping robot.")
                self.stop_robot()
                rate.sleep()
                continue

            # Get the current pose from file
            current_pose_string = ""
            current_pose = []
            with open('forward_kinematics', 'r') as file:
                current_pose_string = file.read().strip()
            if current_pose_string:
                current_pose = eval(current_pose_string.strip()) 
            else:
                continue
            # print("current_pose:", current_pose)

            current_x, current_y, current_z = current_pose[0], current_pose[1], current_pose[2]

            # Get the target pose
            target_x, target_y, target_z = self.buffered_way_points[0]
            # print(target_x)
            # print(target_y)
            # print(target_z)

            # Calculate the difference
            diff_x = target_x - current_x
            diff_y = target_y - current_y
            diff_z = target_z - current_z

            # Calculate the distance to the target
            distance = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)

            # Check if the target is reached
            if distance < threshold:
                rospy.loginfo("Target reached. Removing waypoint from buffer.")
                self.buffered_way_points.pop(0)
                continue

            # Calculate normalized velocity components
            velocity_x = (diff_x / distance) * speed_factor
            velocity_y = (diff_y / distance) * speed_factor
            velocity_z = (diff_z / distance) * speed_factor

            # Create and publish the velocity command
            twist_command = self.FillTwistCommand(0, velocity_x, velocity_y, velocity_z, 0.0, 0.0, 0.0, 0)

            self.buffered_way_points.clear()  # Clear buffered waypoints after sending
            self.buffered_msg_ids.clear()  # Clear buffered timestamp after sending

            self.velocity_publisher.publish(twist_command)
            rate.sleep()


    def stop_robot(self):
        # rospy.loginfo("Stopping the robot...")
        stop_command = TwistCommand()
        stop_command.reference_frame = 0
        stop_command.twist.linear_x = 0.0
        stop_command.twist.linear_y = 0.0
        stop_command.twist.linear_z = 0.0
        stop_command.twist.angular_x = 0.0
        stop_command.twist.angular_y = 0.0
        stop_command.twist.angular_z = 0.0
        stop_command.duration = 0
        self.velocity_publisher.publish(stop_command)
    

    def main(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/waypoint_action_python")
        except:
            pass

        if success:
            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            #*******************************************************************************
            
            #*******************************************************************************
            # Activate the action notifications
            success &= self.example_subscribe_to_a_robot_notification()
            #*******************************************************************************

            #*******************************************************************************
            # Move the robot to the Home position with an Action
            success &= self.example_home_the_robot()
            #*******************************************************************************

            buffer_thread = Thread(target=self.buffer_pose)
            publish_velocity_thread = Thread(target=self.publish_velocity)

            buffer_thread.start()
            publish_velocity_thread.start()

            buffer_thread.join()
            publish_velocity_thread.join()

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/waypoint_action_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")


if __name__ == "__main__":
    ex = WaypointActionClient()
    ex.main()
