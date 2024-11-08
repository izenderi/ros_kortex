import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraInfoProcessor:
    def __init__(self):
        rospy.init_node('camera_info_processor')
        self.bridge = CvBridge()
        
        # Subscriber for CameraInfo and Image messages
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        
        # Initialize variables
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.image = None

    def camera_info_callback(self, camera_info):
        # Extract necessary parameters from CameraInfo message
        self.fx = camera_info.K[0]  # Focal length in x-direction
        self.fy = camera_info.K[4]  # Focal length in y-direction
        self.cx = camera_info.K[2]  # Optical center in x-direction
        self.cy = camera_info.K[5]  # Optical center in y-direction

    def image_callback(self, image_msg):
        # Convert ROS Image message to OpenCV image
        self.image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        
        # Example pixel coordinates (replace with actual detection logic)
        pixel_x = 722  # Example pixel x-coordinate
        pixel_y = 326 # Example pixel y-coordinate
        
        # Draw circle at pixel coordinates on the image
        if self.image is not None:
            cv2.circle(self.image, (pixel_x, pixel_y), 5, (0, 255, 0), -1)  # Green circle at (pixel_x, pixel_y)
        
            # Display the image with pixel coordinates
            cv2.imshow("Image with Pixel Coordinates", self.image)
            cv2.waitKey(1)  # Adjust according to your preference

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CameraInfoProcessor()
        node.run()
    except rospy.ROSInterruptException:
        pass
