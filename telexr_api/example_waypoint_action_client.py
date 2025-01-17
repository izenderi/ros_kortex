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

import actionlib

from kortex_driver.srv import *
from kortex_driver.msg import *

class ExampleWaypointActionClient:
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

            # <RTEN> circle hard code
            # Publisher for velocity commands
            self.velocity_publisher = rospy.Publisher(
                '/my_gen3/in/cartesian_velocity',
                TwistCommand,
                queue_size=10
            )
            # Parameters for the circle
            self.radius = 0.1  # Radius of the circle in meters
            self.center_y = 0.0  # Center of the circle in the y-axis
            self.center_z = 0.4  # Center of the circle in the z-axis
            self.angular_velocity = 0.5  # Angular velocity in radians per second
            self.frequency = 20  # Frequency of publishing velocity commands (Hz)
            self.duration = 10  # Duration to complete the circle in seconds
            self.constant_x_velocity = 0.0  # Velocity in the x-direction (optional)
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
    
    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if self.last_action_notif_type == ActionEvent.ACTION_END:
                # rospy.loginfo("Received ACTION_END notification")
                return True
            elif self.last_action_notif_type == ActionEvent.ACTION_ABORT:
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            time.sleep(0.001)

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
        
    def r10_example_cartesian_waypoint_action(self):
        self.last_action_notif_type = None

        client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', kortex_driver.msg.FollowCartesianTrajectoryAction)

        client.wait_for_server()

        goal = FollowCartesianTrajectoryGoal()

        config = self.get_product_configuration()
# Publisher for velocity co# Parameters for the circle
        self.radius = 0.1  # Radius of the circle in meters
        self.center_y = 0.0  # Center of the circle in the y-axis
        self.center_z = 0.4  # Center of the circle in the z-axis
        self.angular_velocity = 1.0  # Angular velocity in radians per second
        self.frequency = 20  # Frequency of publishing velocity commands (Hz)
        self.duration = 10  # Duration to complete the circle in seconds
        self.constant_x_velocity = 0.0  # Velocity in the x-direction (optional)mmands
        self.velocity_publisher = rospy.Publisher(
            '/my_gen3/in/cartesian_velocity',
            TwistCommand,
            queue_size=10
        )
        if config.output.model == ModelId.MODEL_ID_L31:
            
            goal.trajectory.append(self.FillCartesianWaypoint(0.439,  0.194,  0.448, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.200,  0.150,  0.400, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.350,  0.050,  0.300, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
        else:
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.5,  math.radians(90), 0, math.radians(90), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.33, math.radians(90), 0, math.radians(90), 0.1))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.48,  0.33, math.radians(90), 0, math.radians(90), 0.1))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.61, 0.22,  0.4,  math.radians(90), 0, math.radians(90), 0.1))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.48,  0.33, math.radians(90), 0, math.radians(90), 0.1))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.63, -0.22, 0.45, math.radians(90), 0, math.radians(90), 0.1))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.65, 0.05,  0.45, math.radians(90), 0, math.radians(90), 0))

            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.5,  math.radians(90), 0, math.radians(90), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.4,  math.radians(90), 0, math.radians(90), 0.01))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.33, math.radians(90), 0, math.radians(90), 0.01))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.4,  math.radians(90), 0, math.radians(90), 0.01))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.5,  math.radians(90), 0, math.radians(90), 0.01))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.4,  math.radians(90), 0, math.radians(90), 0.01))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.33, math.radians(90), 0, math.radians(90), 0.01))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.4,  math.radians(90), 0, math.radians(90), 0.01))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.5,  math.radians(90), 0, math.radians(90), 0.01))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.4,  math.radians(90), 0, math.radians(90), 0.01))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.33, math.radians(90), 0, math.radians(90), 0.01))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.4,  math.radians(90), 0, math.radians(90), 0.01))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.7,  0.0,   0.5,  math.radians(90), 0, math.radians(90), 0))

            # Parameters for the circle
            radius = 0.1  # Radius of the circle
            center_y = 0.0  # Center of the circle in the y-axis
            center_z = 0.4  # Center of the circle in the z-axis
            constant_x = 0.98  # Fixed x-coordinate
            num_waypoints = 30  # Number of waypoints along the circle

            # Generate circular trajectory waypoints
            for i in range(num_waypoints):
                theta = 2 * math.pi * i / num_waypoints  # Angle for the current waypoint
                y = center_y + radius * math.cos(theta)  # y-coordinate
                z = center_z + radius * math.sin(theta)  # z-coordinate
                if i==0 or i==num_waypoints-1:
                    goal.trajectory.append(self.FillCartesianWaypoint(constant_x, y, z, math.radians(90), 0, math.radians(90), 0.0))
                else:
                    goal.trajectory.append(self.FillCartesianWaypoint(constant_x, y, z, math.radians(90), 0, math.radians(90), 0.01))

        for i in range(1):

            # Call the service
            # rospy.loginfo("Sending goal(Cartesian waypoint) to action server...")
            try:
                client.send_goal(goal)
            except rospy.ServiceException:
                rospy.logerr("Failed to send goal.")
                return False
            else:
                # time.sleep(0.001)
                client.wait_for_result()
                # self.wait_for_action_end_or_abort()
                # return True
        return True
    
    def publish_velocity(self):
        rate = rospy.Rate(self.frequency)
        steps = int(self.frequency * self.duration)
        delta_t = 1 / self.frequency

        rospy.loginfo("Starting circular velocity control...")
        for i in range(steps):
            theta = self.angular_velocity * i * delta_t

            # Calculate linear velocities for circular motion
            linear_y = -self.radius * self.angular_velocity * math.sin(theta)  # dy/dt
            linear_z = self.radius * self.angular_velocity * math.cos(theta)   # dz/dt

            # Create the velocity command
            twist_command = TwistCommand()
            twist_command.reference_frame = 0  # Base reference frame
            twist_command.twist.linear_x = self.constant_x_velocity
            twist_command.twist.linear_y = linear_y
            twist_command.twist.linear_z = linear_z
            twist_command.twist.angular_x = 0.0
            twist_command.twist.angular_y = 0.0
            twist_command.twist.angular_z = 0.0
            twist_command.duration = 0  # Continuous execution

            # Publish the velocity command
            self.velocity_publisher.publish(twist_command)
            rate.sleep()

        # Stop the robot after completing the circle
        self.stop_robot()

    def stop_robot(self):
        rospy.loginfo("Stopping the robot...")
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

            #*******************************************************************************
            # Example of Cartesian waypoint using an action client
            # success &= self.example_cartesian_waypoint_action()
            success &= self.r10_example_cartesian_waypoint_action()
            # self.publish_velocity()
            #*******************************************************************************

            #*******************************************************************************
            # Move back the robot to the Home position with an Action
            # success &= self.example_home_the_robot()
            #*******************************************************************************

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/waypoint_action_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")


if __name__ == "__main__":
    ex = ExampleWaypointActionClient()
    ex.main()
