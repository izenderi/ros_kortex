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

import roslibpy
import json

from threading import Thread

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

            # Initialize attributes for buffered waypoints
            self.buffered_way_points = []
            self.buffered_msg_ids = []
            self.last_action_notif_type = None
            self.all_notifs_succeeded = True

            # for rosbridge
            self.user_pose = []
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
            
    def parse_message(self, message):
        listener_start = time.time()

        parsed_message = json.loads(message)
        message_id = parsed_message['message_id']
        fused_pose = parsed_message['fused_pose']
        time1 = parsed_message['time1']

        self.user_pose = eval(fused_pose)
        self.message_id = message_id

        # write to file forward_kinematics
        with open('time1', 'w') as file:
            file.write(str(time1))

        # with open('listen_pose', 'w') as file:
        #     file.write(str(fused_pose))

        with open('msg_id', 'w') as file:
            file.write(str(message_id))

    def listen_from_xr(self, xr, port):
        client = roslibpy.Ros(host=xr, port=port)
        client.run()

        listener = roslibpy.Topic(client, '/chatter', 'std_msgs/String')
        listener.subscribe(lambda message: self.parse_message(message['data']))

        try:
            while not rospy.is_shutdown():
                if self.user_pose:
                    x = 0.5 + self.user_pose[2] * 1
                    y = 0.0 + self.user_pose[0] * -1
                    z = 0.5 + self.user_pose[1] * 1

                    if [x,y,z] not in self.buffered_way_points:
                        self.buffered_way_points.append([x, y, z])
                # print("Len of buffer: ", len(self.buffered_way_points))
        except KeyboardInterrupt:
            client.terminate()
        
    def r10_example_cartesian_waypoint_action(self):

        self.last_action_notif_type = None

        client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', kortex_driver.msg.FollowCartesianTrajectoryAction)

        client.wait_for_server()

        goal = FollowCartesianTrajectoryGoal()

        config = self.get_product_configuration()

        while not rospy.is_shutdown():

            # for waypoint in self.buffered_way_points:

            #     if config.output.model == ModelId.MODEL_ID_L31:
            #         goal.trajectory.append(self.FillCartesianWaypoint(waypoint[0], waypoint[1], waypoint[2], math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
            #     else:
            #         goal.trajectory.append(self.FillCartesianWaypoint(waypoint[0], waypoint[1], waypoint[2], math.radians(90), 0, math.radians(90), 0)) # 0 blending is interpolation

            waypoint = self.buffered_way_points[0]
            goal.trajectory.append(self.FillCartesianWaypoint(waypoint[0], waypoint[1], waypoint[2], math.radians(90), 0, math.radians(90), 0))

            self.buffered_way_points.clear()  # Clear buffered waypoints after sending
            self.buffered_msg_ids.clear()  # Clear buffered timestamp after sending

            # Call the service
            # rospy.loginfo("Len of goal:", len(goal))
            # rospy.loginfo(f"Sending goal to action server...")
            try:
                client.send_goal(goal)
            except rospy.ServiceException:
                rospy.logerr("Failed to send goal.")
                return False
            else:
                # time.sleep(0.001)
                client.wait_for_result()
                # self.wait_for_action_end_or_abort()
                goal = FollowCartesianTrajectoryGoal()
                # return True
                # rate.sleep()

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

            rospy.loginfo("Setup complete!")

            listen_thread = Thread(target=self.listen_from_xr, args=("10.13.145.127", 9090))
            move_thread = Thread(target=self.r10_example_cartesian_waypoint_action)

            listen_thread.start()
            time.sleep(3)
            move_thread.start()

            listen_thread.join()
            move_thread.join()

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/waypoint_action_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")


if __name__ == "__main__":
    ex = ExampleWaypointActionClient()
    ex.main()
