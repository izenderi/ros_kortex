#!/usr/bin/env python3

import time
import csv
import rospy
import numpy as np
from sensor_msgs.msg import JointState

class ForwardKinematics:
    def __init__(self):
        rospy.init_node('forward_kinematics')
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        rospy.loginfo("Using robot_name " + self.robot_name)
        self.joint_state_sub = rospy.Subscriber("/" + self.robot_name + "/joint_states", JointState, self.cb_joint_state)
        self.joint_angles = None

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

    def main(self):

        data_log = []
        output_file = 'forward.csv'

        rospy.loginfo("Waiting for joint states...")
        rate = rospy.Rate(80)  # 80 Hz
        # while not rospy.is_shutdown() and self.joint_angles is None:

        try:
            while not rospy.is_shutdown():
                rate.sleep()  # Wait for joint state callback to receive data

                if self.joint_angles:

                    position, orientation = self.forward_kinematics(self.joint_angles)
                    position = self.error_offset(position)

                    # write to file forward_kinematics
                    with open('forward_kinematics', 'w') as file:
                        file.write(f"[{position[0]:.6f}, {position[1]:.6f}, {position[2]:.6f}]")  # Convert list to string and write it to the file
                        # file.write(str(position))

                    time1 = time.time()
                    data_string = f"[{position[0]}, {position[1]}, {position[2]}, 0.0, 0.0, 0.0, 0.0]"

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

if __name__ == "__main__":
    fk = ForwardKinematics()
    fk.main()