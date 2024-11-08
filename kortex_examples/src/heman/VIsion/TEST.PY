#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
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
        if self.latest_image is not None and self.latest_point_cloud is not None:
            image_height, image_width, _ = self.latest_image.shape

            for obj in self.detected_objects:
                xmin = obj.xmin
                ymin = obj.ymin
                xmax = obj.xmax
                ymax = obj.ymax

                center_x = (xmin + xmax) // 2
                center_y = (ymin + ymax) // 2

                if center_x < 0 or center_x >= image_width or center_y < 0 or center_y >= image_height:
                    rospy.logerr(f"Object center ({center_x}, {center_y}) is out of image bounds")
                    continue

                cv2.rectangle(self.latest_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.circle(self.latest_image, (center_x, center_y), 5, (0, 0, 255), -1)
                label = f"{obj.Class} ({center_x}, {center_y})"
                cv2.putText(self.latest_image, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                points = []
                for p in pc2.read_points(self.latest_point_cloud, field_names=("x", "y", "z"), skip_nans=True):
                    u = int((p[0] * self.fx) / p[2] + self.cx)
                    v = int((p[1] * self.fy) / p[2] + self.cy)
                    if xmin <= u <= xmax and ymin <= v <= ymax:
                        points.append(p)

                if points:
                    points = np.array(points)
                    min_depth = np.min(points[:, 2])
                    max_depth = np.max(points[:, 2])
                    object_height = max_depth - min_depth
                    rospy.loginfo(f"Object {obj.Class} has min depth: {min_depth:.2f}, max depth: {max_depth:.2f}, and height: {object_height:.2f} meters")

                    X = (center_x - self.cx) * min_depth / self.fx
                    Y = (center_y - self.cy) * min_depth / self.fy
                    Z = min_depth

                    camera_coords = np.array([X, Y, Z])
                    robot_base_coords = self.transform_to_robot_base(camera_coords)
                    if robot_base_coords is not None:
                        X_robot, Y_robot, Z_robot = robot_base_coords
                        rospy.loginfo(f"Object {obj.Class} at pixel coordinates ({center_x}, {center_y}) has 3D coordinates in camera frame ({X:.2f}, {Y:.2f}, {Z:.2f}) and in robot base frame ({X_robot:.2f}, {Y_robot:.2f}, {Z_robot:.2f})")

                        if self.is_stable_coordinate(robot_base_coords):
                            self.publish_transformed_point(robot_base_coords, min_depth, self.target_frame)
                            rospy.wait_for_service('movement_complete')

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

