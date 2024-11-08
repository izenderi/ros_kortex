import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import cv2
import numpy as np

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

        # Publisher for object coordinates
        self.coordinates_pub = rospy.Publisher('/object_coordinates', PointStamped, queue_size=10)

        # Initialize variables
        self.latest_image = None
        self.fx = 0.0  # focal length in x
        self.fy = 0.0  # focal length in y
        self.cx = 0.0  # optical center in x
        self.cy = 0.0  # optical center in y

    def camera_info_callback(self, camera_info):
        # Extract camera intrinsic parameters from CameraInfo message
        self.fx = camera_info.K[0]
        self.fy = camera_info.K[4]
        self.cx = camera_info.K[2]
        self.cy = camera_info.K[5]

    def image_callback(self, ros_image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            self.detect_blue_object()  # Detect blue object in the image

        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def detect_blue_object(self):
        if self.latest_image is not None:
            # Convert image to HSV color space
            hsv_image = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)

            # Define range of blue color in HSV
            lower_blue = np.array([100, 100, 100])  # Lower bound for blue
            upper_blue = np.array([140, 255, 255])  # Upper bound for blue

            # Threshold the HSV image to get only blue colors
            mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(self.latest_image, self.latest_image, mask=mask)

            # Find contours of blue regions
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                # Get bounding box coordinates
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate center of the bounding box
                center_x = x + w // 2
                center_y = y + h // 2

                # Draw bounding box around blue object
                cv2.rectangle(self.latest_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

                # Draw center point on the image
                cv2.circle(self.latest_image, (center_x, center_y), 5, (0, 255, 0), -1)

                # Publish center coordinates
                self.publish_coordinates(center_x, center_y)

            # Display the image with bounding box and center point
            cv2.imshow("Blue Object Detection", self.latest_image)
            cv2.waitKey(1)  # Adjust according to your preference

    def publish_coordinates(self, x, y):
        # Publish PointStamped message with coordinates
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "camera_link"  # Update with your camera frame
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.0  # Assuming z-coordinate is 0 for a 2D movement

        self.coordinates_pub.publish(point_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = ObjectDetectionNode()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
