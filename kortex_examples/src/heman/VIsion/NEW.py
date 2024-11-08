#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Point, PointStamped

class CoordinateTransformerNode:
    def __init__(self):
        rospy.init_node('coordinate_transformer_node', anonymous=True)

        self.tf_listener = tf.TransformListener()
        self.source_frame = rospy.get_param('~source_frame', 'camera_link')  # Camera frame
        self.target_frame = rospy.get_param('~target_frame', 'base_link')    # Robot base frame

        # Publisher to output the transformed point
        self.point_pub = rospy.Publisher('transformed_point', PointStamped, queue_size=10)

        rospy.loginfo("Coordinate Transformer Node Initialized")

    def transform_to_robot_base(self, camera_coords):
        try:
            self.tf_listener.waitForTransform(self.target_frame, self.source_frame, rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.target_frame, self.source_frame, rospy.Time(0))
            rospy.loginfo("Translation: %s", str(trans))
            rospy.loginfo("Rotation (quaternion): %s", str(rot))

            R = tf.transformations.quaternion_matrix(rot)[:3, :3]
            T = np.array(trans)

            rospy.loginfo("Rotation matrix:\n%s", str(R))
            rospy.loginfo("Translation vector: %s", str(T))

            # Apply rotation and translation
            robot_base_coords = np.dot(R, camera_coords) + T
            return robot_base_coords
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Exception: %s", str(e))
            return None

    def publish_transformed_point(self, transformed_coords):
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = self.target_frame
        point_msg.point = Point(*transformed_coords)
        self.point_pub.publish(point_msg)
        rospy.loginfo("Published transformed point: x=%.6f, y=%.6f, z=%.6f", point_msg.point.x, point_msg.point.y, point_msg.point.z)

if __name__ == "__main__":
    transformer_node = CoordinateTransformerNode()

    # Example usage in the same script
    rospy.sleep(2)  # Wait a bit for the TF listener to get the transforms

    # Coordinates in the camera frame (in meters)
    x, y, z = 0.04163,0.00741,0.36
    camera_coords = np.array([x, y, z])
    rospy.loginfo("Camera frame coordinates: x=%.6f, y=%.6f, z=%.6f", x, y, z)

    transformed_coords = transformer_node.transform_to_robot_base(camera_coords)
    if transformed_coords is not None:
        rospy.loginfo("Transformed Coordinates in Robot Base Frame: x=%.6f, y=%.6f, z=%.6f", transformed_coords[0], transformed_coords[1], transformed_coords[2])
        transformer_node.publish_transformed_point(transformed_coords)

    rospy.spin()