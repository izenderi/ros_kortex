import rospy
import tf
from geometry_msgs.msg import PointStamped

def transform_coordinates(camera_x, camera_y, camera_z):
    # Initialize the ROS node
    rospy.init_node('coordinate_transformer', anonymous=True)
    
    # Create a tf listener
    listener = tf.TransformListener()

    try:
        # Wait for the transform to become available
        listener.waitForTransform('/base_link', '/camera_color_frame', rospy.Time(), rospy.Duration(10.0))
        rospy.loginfo("Transform available between /base_link and /camera_color_frame")

        # Define the point in the camera frame
        camera_point = PointStamped()
        camera_point.header.frame_id = 'camera_color_frame'
        camera_point.header.stamp = rospy.Time(0)  # Use time 0 to get the latest transform
        camera_point.point.x = camera_x
        camera_point.point.y = camera_y
        camera_point.point.z = camera_z

        # Transform the point to the robot base frame
        base_point = listener.transformPoint('base_link', camera_point)

        # Extract the transformed coordinates
        X_base = base_point.point.x
        Y_base = base_point.point.y
        Z_base = base_point.point.z

        # Print the transformed coordinates
        rospy.loginfo(f"Transformed coordinates: X={X_base}, Y={Y_base}, Z={Z_base}")
        print(f"Transformed coordinates: X={X_base}, Y={Y_base}, Z={Z_base}")

        return X_base, Y_base, Z_base

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"Error transforming coordinates: {e}")
        return None, None, None

if __name__ == '__main__':
    # Example camera coordinates
    camera_coords = (0.00335,0.00242,0.36)
    X_base, Y_base, Z_base = transform_coordinates(*camera_coords)
    
    if X_base is not None:
        rospy.loginfo(f"Transformed coordinates: X={X_base}, Y={Y_base}, Z={Z_base}")
