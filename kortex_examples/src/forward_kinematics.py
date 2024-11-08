#!/usr/bin/env python

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

   

    def main(self):
        rospy.loginfo("Waiting for joint states...")
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown() and self.joint_angles is None:
            rate.sleep()  # Wait for joint state callback to receive data

        if self.joint_angles:
            position, orientation = self.forward_kinematics(self.joint_angles)
            #new_joint_angles_r = self.inverse_kinematics(position, orientation, self.joint_angles)
            #new_joint_angles_d = [self.radians_to_degrees(angle) for angle in new_joint_angles_r]
            
            rospy.loginfo("Position of the end effector: %s", position)
            rospy.loginfo("Orientation of the end effector (rotation matrix): %s", orientation)
            #rospy.loginfo("New Joint Angles (Radians): %s", new_joint_angles_r)
            #rospy.loginfo("New_joint_angles_degrees: %s", new_joint_angles_d)

        else:
            rospy.logerr("Failed to get joint states.")

if __name__ == "__main__":
    fk = ForwardKinematics()
    fk.main()