#!/usr/bin/env python3

import csv
import rospy
import time
import math
from threading import Thread

import roslibpy
import json

import actionlib

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
            self.buffered_msg_ids = []
            self.last_action_notif_type = None
            self.all_notifs_succeeded = True

            # outut csv
            self.data_log = []
            self.output_file = 'data.csv'

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

            # Initialize attributes for buffered waypoints
            self.buffered_way_points = []
            self.buffered_msg_ids = []
            self.last_action_notif_type = None
            self.all_notifs_succeeded = True

            # for rosbridge
            self.user_pose = []
            
            self.is_init_success = True
        except:
            self.is_init_success = False

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

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

    def move_to_buffered_poses(self):
        count = 0
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

            for msg_id, waypoint in zip(self.buffered_msg_ids, self.buffered_way_points):
                constrained_pose = ConstrainedPose()
                # constrained_pose = CartesianWaypoint()

                # constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

                constrained_pose.target_pose.x = waypoint[0]
                constrained_pose.target_pose.y = waypoint[1]
                constrained_pose.target_pose.z = waypoint[2]
                constrained_pose.target_pose.theta_x = 90
                constrained_pose.target_pose.theta_y = 0
                constrained_pose.target_pose.theta_z = 90
                # constrained_pose.blending_radius = 0.1

                goal.input.oneof_action_parameters.reach_pose.append(constrained_pose)

                self.data_log.append([msg_id, f"[{waypoint[0]}, {waypoint[1]}, {waypoint[2]}, 0.0, 0.0, 0.0, 0.0]"])
            
            self.buffered_way_points.clear()  # Clear buffered waypoints after sending
            self.buffered_msg_ids.clear()  # Clear buffered timestamp after sending

            # rospy.loginfo(f"Sending {len(self.buffered_way_points)} waypoints...")
            self.last_action_notif_type = None

            try:
                self.execute_action(goal)
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to send waypoints: {e}")
                continue
            else:
                self.wait_for_action_end_or_abort()
                count += 1
                if count % 10 == 0:
                    with open(self.output_file, mode='w', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow(["Timestamp", "gt_bot_pose"])
                        writer.writerows(self.data_log)

    def main(self):
        success = self.is_init_success

        if success:
            success &= self.example_clear_faults()
            success &= self.example_home_the_robot()
            success &= self.example_set_cartesian_reference_frame()

            buffer_thread = Thread(target=self.listen_from_xr, args=("10.13.145.127", 9090))
            move_thread = Thread(target=self.move_to_buffered_poses)
            # move_thread = Thread(target=self.r10_move_to_buffered_poses)

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
