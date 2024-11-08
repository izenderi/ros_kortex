#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        
        # Create CvBridge object
        self.bridge = CvBridge()
        
        # Publishers
        self.color_image_pub = rospy.Publisher('/processed/color_image', Image, queue_size=10)
        self.depth_image_pub = rospy.Publisher('/processed/depth_image', Image, queue_size=10)
        self.point_cloud_pub = rospy.Publisher('/processed/point_cloud', PointCloud2, queue_size=10)

        # Subscribers
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_image_callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        rospy.Subscriber('/camera/depth/points', PointCloud2, self.point_cloud_callback)

    def color_image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Optionally process the color image here
            
            # Convert OpenCV image back to ROS Image message
            image_message = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            
            # Publish the color image
            self.color_image_pub.publish(image_message)
        except Exception as e:
            rospy.logerr("Failed to process color image: %s", str(e))

    def depth_image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            
            # Optionally process the depth image here
            
            # Convert OpenCV image back to ROS Image message
            image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
            
            # Publish the depth image
            self.depth_image_pub.publish(image_message)
        except Exception as e:
            rospy.logerr("Failed to process depth image: %s", str(e))

    def point_cloud_callback(self, data):
        try:
            # Publish the point cloud data
            self.point_cloud_pub.publish(data)
        except Exception as e:
            rospy.logerr("Failed to process point cloud: %s", str(e))

if __name__ == '__main__':
    try:
        camera_node = CameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
