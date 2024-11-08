#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes
import cv2

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.detection_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detection_callback)

        self.detected_objects = []
        self.latest_image = None

    def image_callback(self, ros_image):
        try:
            # Convert ROS image message to OpenCV image
            self.latest_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            rospy.loginfo("Image received and converted successfully.")

        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def detection_callback(self, bounding_boxes):
        # Process the detected bounding boxes
        self.detected_objects = bounding_boxes.bounding_boxes

        # Draw bounding boxes on the latest image
        if self.latest_image is not None:
            for obj in self.detected_objects:
                cv2.rectangle(self.latest_image, (obj.xmin, obj.ymin), (obj.xmax, obj.ymax), (0, 255, 0), 2)
                cv2.putText(self.latest_image, f"{obj.Class} {obj.probability:.2f}", (obj.xmin, obj.ymin - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Display the resulting frame
            cv2.imshow('Object Detection', self.latest_image)
            cv2.waitKey(12)

if __name__ == '__main__':
    try:
        detector = ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
