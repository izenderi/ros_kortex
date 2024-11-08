#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
import cv2
import numpy as np
import tf

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)  # Corrected to use color camera info
        self.detection_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detection_callback)

        self.detected_objects = []
        self.latest_image = None
        self.latest_depth = None
        self.fx = 0.0  # focal length in x
        self.fy = 0.0  # focal length in y
        self.cx = 0.0  # optical center in x
        self.cy = 0.0  # optical center in y

        self.tf_listener = tf.TransformListener()
        self.source_frame = 'camera_link'  # Adjust this frame according to your setup
        self.target_frame = 'base_link'  # Adjust this frame according to your setup

    def camera_info_callback(self, camera_info):
        # Extract camera intrinsic parameters from CameraInfo message
        self.fx = camera_info.K[0]
        self.fy = camera_info.K[4]
        self.cx = camera_info.K[2]
        self.cy = camera_info.K[5]

    def image_callback(self, ros_image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            self.draw_detected_objects()  # Draw objects on the image
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

    def depth_callback(self, ros_image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")

            # Convert depth to meters if necessary (assuming input is in millimeters)
            if self.latest_depth.dtype == np.uint16:
                self.latest_depth = self.latest_depth.astype(np.float32) / 1000.0
        except CvBridgeError as e:
            rospy.logerr(f"Error converting depth image: {e}")

    def detection_callback(self, bounding_boxes):
        self.detected_objects = bounding_boxes.bounding_boxes

    def transform_to_robot_base(self, camera_coords):
        try:
            self.tf_listener.waitForTransform(self.target_frame, self.source_frame, rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.target_frame, self.source_frame, rospy.Time(0))
            R = tf.transformations.quaternion_matrix(rot)[:3, :3]
            T = np.array(trans)

            robot_base_coords = R @ camera_coords + T
            return robot_base_coords
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF Exception")
            return None

    def draw_detected_objects(self):
        if self.latest_image is not None and self.latest_depth is not None:
            image_height, image_width, _ = self.latest_image.shape

            # Resize depth image to match color image dimensions
            depth_resized = cv2.resize(self.latest_depth, (image_width, image_height), interpolation=cv2.INTER_NEAREST)

            for obj in self.detected_objects:
                xmin = obj.xmin
                ymin = obj.ymin
                xmax = obj.xmax
                ymax = obj.ymax

                # Calculate object center coordinates
                center_x = (xmin + xmax) // 2
                center_y = (ymin + ymax) // 2

                # Check if center coordinates are within image bounds
                if center_x < 0 or center_x >= image_width or center_y < 0 or center_y >= image_height:
                    rospy.logerr(f"Object center ({center_x}, {center_y}) is out of image bounds")
                    continue

                # Draw bounding box
                cv2.rectangle(self.latest_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

                # Draw red dot at the center of the bounding box
                cv2.circle(self.latest_image, (center_x, center_y), 5, (0, 0, 255), -1)

                # Display class label and pixel coordinates at the center of the bounding box
                label = f"{obj.Class} ({center_x}, {center_y})"
                cv2.putText(self.latest_image, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Get the depth value at the center of the bounding box
                Z = depth_resized[center_y, center_x]

                if Z > 0:  # Ensure depth is valid
                    # Compute the 3D coordinates
                    X = (center_x - self.cx) * Z / self.fx
                    Y = (center_y - self.cy) * Z / self.fy

                    # Transform to robot base frame
                    camera_coords = np.array([X, Y, Z])
                    robot_base_coords = self.transform_to_robot_base(camera_coords)
                    if robot_base_coords is not None:
                        X_robot, Y_robot, Z_robot = robot_base_coords
                        # Print 3D coordinates to console
                        rospy.loginfo(f"Object {obj.Class} at pixel coordinates ({center_x}, {center_y}) has 3D coordinates in camera frame ({X:.2f}, {Y:.2f}, {Z:.2f}) and in robot base frame ({X_robot:.2f}, {Y_robot:.2f}, {Z_robot:.2f})")

            # Display image with annotations
            cv2.imshow('Object Detection', self.latest_image)
            cv2.waitKey(1)
        else:
            rospy.logerr("No image or depth data received")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = ObjectDetectionNode()
        detector.run()
    except rospy.ROSInterruptException:
        pass
