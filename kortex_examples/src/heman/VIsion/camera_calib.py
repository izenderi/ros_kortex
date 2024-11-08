#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraCalibrationNode:
    def __init__(self):
        rospy.init_node('camera_calibration_node', anonymous=True)
        self.bridge = CvBridge()

        # Initialize camera matrix parameters
        self.camera_matrix = None
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            rospy.loginfo("Camera matrix received.")

            # Perform camera calibration (optional: print or save camera matrix)
            rospy.loginfo(f"Camera Matrix (K):\n{self.camera_matrix}")

            # Save camera calibration parameters (optional)
            np.savez("camera_calibration.npz", mtx=self.camera_matrix)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        calibrator = CameraCalibrationNode()
        calibrator.run()
    except rospy.ROSInterruptException:
        pass
