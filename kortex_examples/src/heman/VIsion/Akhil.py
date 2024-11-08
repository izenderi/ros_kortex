#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, JointState
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
import cv2
import numpy as np
import tf
import sensor_msgs.point_cloud2 as pc2
from collections import deque
from std_srvs.srv import Empty

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        self.detection_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detection_callback)
        self.point_cloud_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.point_cloud_callback)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        self.point_pub = rospy.Publisher('/detected_object_point', PointStamped, queue_size=10)

        self.detected_objects = []
        self.latest_image = None
        self.latest_depth = None
        self.latest_point_cloud = None
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0

        self.tf_listener = tf.TransformListener()
        self.source_frame = 'camera_link'
        self.target_frame = 'base_link'

        self.joint_names = []
        self.joint_positions = []
        self.stable_threshold = 5
        self.stable_coordinates = deque(maxlen=self.stable_threshold)
        self.movement_complete_service = rospy.ServiceProxy('movement_complete', Empty)

    def camera_info_callback(self, camera_info):
        self.fx = camera_info.K[0]
        self.fy = camera_info.K[4]
        self.cx = camera_info.K[2]
        self.cy = camera_info.K[5]

    def image_callback(self, ros_image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            self.draw_detected_objects()
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

    def depth_callback(self, ros_image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
            if self.latest_depth.dtype == np.uint16:
                self.latest_depth = self.latest_depth.astype(np.float32) / 1000.0
        except CvBridgeError as e:
            rospy.logerr(f"Error converting depth image: {e}")

    def point_cloud_callback(self, point_cloud):
        self.latest_point_cloud = point_cloud

    def detection_callback(self, bounding_boxes):
        self.detected_objects = bounding_boxes.bounding_boxes

    def joint_state_callback(self, joint_state):
        self.joint_names = joint_state.name
        self.joint_positions = joint_state.position
        self.print_joint_angles()

    def print_joint_angles(self):
        rospy.loginfo("Current Joint Angles:")
        for name, position in zip(self.joint_names, self.joint_positions):
            rospy.loginfo(f"Joint {name}: {np.degrees(position):.2f} degrees")

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

    def publish_transformed_point(self, point, min_depth, frame_id):
        point_stamped = PointStamped()
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.header.frame_id = frame_id
        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_stamped.point.z = min_depth  # Publish min_depth here
        self.point_pub.publish(point_stamped)

    def is_stable_coordinate(self, coord):
        self.stable_coordinates.append(coord)
        if len(self.stable_coordinates) == self.stable_threshold:
            return all(np.array_equal(coord, self.stable_coordinates[0]) for coord in self.stable_coordinates)
        return False

    def draw_detected_objects(self):
        if self.latest_image is not None:
            # Convert the captured RGB image to HSV color space
            hsv_image = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)

            # Define the red color range in HSV
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])

            # Create masks for the red color
            mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)

            # Clean the mask using morphological operations
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

            # Use the Canny edge detector to find edges in the red mask
            edges = cv2.Canny(red_mask, 50, 150)

            # Use Hough Line Transform to detect lines in the edge-detected image
            lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

            # Draw the detected lines on the original image
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(self.latest_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Display the results
            cv2.imshow('Detected Red Line', self.latest_image)
            cv2.waitKey(1)
        else:
            rospy.logerr("No image data received")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = ObjectDetectionNode()
        detector.run()
    except rospy.ROSInterruptException:
        pass
