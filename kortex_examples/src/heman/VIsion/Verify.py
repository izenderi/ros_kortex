import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthChecker:
    def __init__(self):
        rospy.init_node('depth_checker')
        self.bridge = CvBridge()
        
        # Subscriber for depth image
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        
        # Initialize depth image variable
        self.depth_image = None
        self.window_name = "Depth Image"

        # Create a named window and set the mouse callback function
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        # Pixel coordinates for which we want to get the depth
        self.target_x = 559
        self.target_y = 332

    def depth_callback(self, depth_msg):
        # Convert ROS Image message to OpenCV depth image
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # Normalize the depth image for display purposes
        depth_display = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

        # Display the depth image
        cv2.imshow(self.window_name, depth_display)
        cv2.waitKey(1)

        # Get and print the depth value at the target pixel coordinates
        self.print_depth_value(self.target_x, self.target_y)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.depth_image is not None:
                # Extract the depth value at the clicked coordinates
                depth_value = self.depth_image[y, x]
                print(f"Depth at pixel ({x}, {y}): {depth_value} meters")

    def print_depth_value(self, x, y):
        if self.depth_image is not None:
            # Extract the depth value at the given coordinates
            depth_value = self.depth_image[y, x]
            print(f"Depth at pixel ({x}, {y}): {depth_value} meters")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DepthChecker()
        node.run()
    except rospy.ROSInterruptException:
        pass
