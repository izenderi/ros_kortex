#!/usr/bin/env python3

import time
import csv
import rospy
import numpy as np
from sensor_msgs.msg import JointState

from threading import Thread

import socket

class ForwardKinematics:
    def __init__(self):
        rospy.init_node('forward_kinematics')
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        rospy.loginfo("Using robot_name " + self.robot_name)
        self.joint_state_sub = rospy.Subscriber("/" + self.robot_name + "/joint_states", JointState, self.cb_joint_state)
        self.joint_angles = None

        self.position_string = "" # storage for latest position

        self.freq = 100
        self.period = 1/self.freq

    def cb_joint_state(self, joint_state):
        self.joint_angles = joint_state.position

    def dh_transform(self, alpha, a, d, theta):
        """
        Create the transformation matrix using DH parameters.
        """
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joint_angles):
        """
        Compute the forward kinematics for the given joint angles.
        """
        # Define the DH parameters of your robot
        dh_params = [
            [np.pi, 0.0, 0.0, 0],                           # Joint 0 (Base)
            [np.pi/2, 0.0, -(0.1564 + 0.1284), joint_angles[0]],         # Joint 1
            [np.pi/2, 0.0, -(0.0054 + 0.0064), joint_angles[1] + np.pi],# Joint 2
            [np.pi/2, 0.0, -(0.2104 + 0.2104), joint_angles[2] + np.pi],# Joint 3
            [np.pi/2, 0.0, -(0.0064 + 0.0064), joint_angles[3] + np.pi],# Joint 4
            [np.pi/2, 0.0, -(0.2084 + 0.1059), joint_angles[4] + np.pi],# Joint 5
            [np.pi/2, 0.0, 0.0, joint_angles[5] + np.pi],                # Joint 6
            [np.pi, 0.0, -(0.1059 + 0.0615), joint_angles[6] + np.pi]   # Joint 7 (End-effector)
        ]

        # Initialize transformation matrix as an identity matrix
        T = np.eye(4)
        
        # Multiply the transformation matrices for each joint
        for i in range(len(dh_params)):
            alpha, a, d, theta = dh_params[i]
            T = np.dot(T, self.dh_transform(alpha, a, d, theta))
        
        # Extract the position and orientation
        position = T[:3, 3]
        orientation = T[:3, :3]
        
        return position, orientation
    
    def error_offset(self, position):
        """
        <RTEN> TODO
        Because the errors are generated from the above forward kinematics
        We mannually change back the error in this function
        """
        position[0] += 0.20174659
        position[1] += 0.0062074
        position[2] += 0.00068264
        return position
    
    def main_fk(self):
        data_log = []
        output_file = 'forward.csv'

        rospy.loginfo("Waiting for joint states...")
        rate = rospy.Rate(self.freq)  # 100 Hz
        # while not rospy.is_shutdown() and self.joint_angles is None:

        try:
            while not rospy.is_shutdown():
                rate.sleep()  # Wait for joint state callback to receive data

                if self.joint_angles:

                    time0 = time.time()

                    position, orientation = self.forward_kinematics(self.joint_angles)
                    position = self.error_offset(position)

                    # write to file forward_kinematics
                    with open('forward_kinematics', 'w') as file:
                        file.write(f"[{position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}]")  # Convert list to string and write it to the file
                        # file.write(str(position))

                    time1 = time.time()
                    # print("Exe Time:", str(time1-time0))
                    data_string = f"[{position[0]}, {position[1]}, {position[2]}, 0.0, 0.0, 0.0, 0.0]"
                    self.position_string = data_string

                    data_log.append([time1, data_string])

                else:
                    rospy.logerr("Failed to get joint states.")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            print("\nSaving data to CSV...")
            with open(output_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Timestamp", "gt_bot_pose"])
                writer.writerows(data_log)
            print(f"Data saved to {output_file}. Exiting...")
    
    def robot_to_xr(self, robot, port, xr, period=1):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((xr, port))

            prev_message_id = -1
            time1_string = ""
            
            while True:
                with open('msg_id', 'r') as file:
                    msg_id = file.read().strip()
                    if msg_id != '':
                        message_id = int(msg_id)

                if prev_message_id < message_id:
                    print(message_id)
                    prev_message_id = message_id
                    # Get the current pose from file
                    with open('time1', 'r') as file:
                        time1_string = file.read().strip()

                data = f"{time1_string}:{self.position_string}"
                s.sendall(data.encode())
                time.sleep(period)

    def main(self):

        with open('msg_id', 'w') as file:
            file.write('0')
        
        fk_thread = Thread(target=self.main_fk)
        robot_to_xr_thread = Thread(target=self.robot_to_xr, args=("", 9091, "10.13.146.99", self.period)) # 0.05 = 50ms period

        fk_thread.start()
        robot_to_xr_thread.start()

        fk_thread.join()
        robot_to_xr_thread.join()

if __name__ == "__main__":
    fk = ForwardKinematics()
    fk.main()