#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class BottleDetectorROS:
    def __init__(self):
        rospy.init_node('bottle_detector', anonymous=True)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_and_display(cv_image)
        except CvBridgeError as e:
            rospy.logerr(e)

    def detect_and_display(self, image):
        # Convert image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply thresholding
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Process each contour
        for cnt in contours:
            # Approximate contour
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            
            # Check if contour resembles a bottle (example condition: number of vertices)
            if len(approx) == 8:  # Adjust condition based on your bottle shape characteristics
                # Get centroid of contour
                M = cv2.moments(approx)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = 0, 0
                
                # Draw contour and centroid
                cv2.drawContours(image, [approx], 0, (0, 255, 0), 2)
                cv2.circle(image, (cx, cy), 5, (255, 0, 0), -1)
                
                # Label contour with centroid coordinates
                label = f"({cx}, {cy})"
                cv2.putText(image, label, (cx - 50, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Display the image with contours and labels
        cv2.imshow('Image', image)
        cv2.waitKey(1)

    def run(self):
        rospy.loginfo("Bottle detector node running...")
        rospy.spin()

if __name__ == '__main__':
    bd = BottleDetectorROS()
    bd.run()
