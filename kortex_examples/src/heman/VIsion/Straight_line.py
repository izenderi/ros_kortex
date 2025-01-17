#!/usr/bin/env python

import rospy
import time
import math
import actionlib
import numpy as np

from kortex_driver.srv import *
from kortex_driver.msg import *

class ExampleWaypointActionClient:
    def __init__(self):
        try:
            rospy.init_node('example_waypoint_action_python')

            self.HOME_ACTION_IDENTIFIER = 2

            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name {}, robot has {} degrees of freedom, and is_gripper_present is {}".format(self.robot_name, self.degrees_of_freedom, self.is_gripper_present))

            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

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
            
            self.is_init_success = True  # Initialize success flag here

        except rospy.ROSException as e:
            rospy.logerr("ROS Initialization failed: {}".format(e))
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
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call OnNotificationActionTopic: {}".format(e))
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")
            rospy.sleep(1.0)
            return True

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call ClearFaults: {}".format(e))
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call ReadAction: {}".format(e))
            return False
        else:
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call ExecuteAction: {}".format(e))
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_cartesian_waypoint_action(self):
        self.last_action_notif_type = None

        client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', FollowCartesianTrajectoryAction)

        client.wait_for_server()

        goal = FollowCartesianTrajectoryGoal()

        # Define the number of points on the circle
        num_points = 20

        # Define circle parameters
        radius = 0.1  # Radius of the circle
        center_x = 0.5  # X coordinate of the circle center
        center_y = 0.0  # Y coordinate of the circle center
        center_z = 0.5  # Z coordinate of the circle center

        # Generate waypoints on the circle
        for i in range(num_points):
            theta = i * (2 * np.pi / num_points)
            x = center_x + radius * np.cos(theta)
            y = center_y + radius * np.sin(theta)
            z = center_z
            waypoint = self.FillCartesianWaypoint(x, y, z, math.radians(90), 0, math.radians(90), blending_radius=0)
            goal.trajectory.append(waypoint)

        rospy.loginfo("Sending goal (Cartesian waypoints in a circle) to action server...")
        try:
            client.send_goal(goal)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to send goal: {}".format(e))
            return False
        else:
            client.wait_for_result()
            return True

    def main(self):
        success = self.is_init_success

        # Attempt to delete existing parameter for clean run
        try:
            rospy.delete_param("/kortex_examples_test_results/waypoint_action_python")
        except KeyError:
            pass

        if success:
            success &= self.example_clear_faults()
            success &= self.example_subscribe_to_a_robot_notification()
            success &= self.example_home_the_robot()
            success &= self.example_cartesian_waypoint_action()
        else:
            rospy.logerr("Initialization failed. Aborting.")

        if not success:
            rospy.set_param("/kortex_examples_test_results/waypoint_action_python", False)
            rospy.logerr("The example encountered an error.")
        else:
            rospy.set_param("/kortex_examples_test_results/waypoint_action_python", True)

if __name__ == "__main__":
    ex = ExampleWaypointActionClient()
    ex.main()
