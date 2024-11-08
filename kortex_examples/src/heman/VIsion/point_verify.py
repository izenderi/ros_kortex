import rospy
import tf
from geometry_msgs.msg import PointStamped

def transform_coordinates(camera_x, camera_y, camera_z):
    rospy.init_node('coordinate_transformer', anonymous=True)
    listener = tf.TransformListener()

    try:
        listener.waitForTransform('/base_link', '/camera_color_frame', rospy.Time(), rospy.Duration(10.0))
        rospy.loginfo("Transform available between /base_link and /camera_color_frame")

        camera_point = PointStamped()
        camera_point.header.frame_id = 'camera_color_frame'
        camera_point.header.stamp = rospy.Time(0)
        camera_point.point.x = camera_x
        camera_point.point.y = camera_y
        camera_point.point.z = camera_z

        base_point = listener.transformPoint('base_link', camera_point)

        X_base = base_point.point.x
        Y_base = base_point.point.y
        Z_base = base_point.point.z

        rospy.loginfo(f"Transformed coordinates: X={X_base}, Y={Y_base}, Z={Z_base}")
        print(f"Transformed coordinates: X={X_base}, Y={Y_base}, Z={Z_base}")

        return X_base, Y_base, Z_base

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"Error transforming coordinates: {e}")
        return None, None, None

def publish_point(x, y, z):
    point_pub = rospy.Publisher('/transformed_point', PointStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = 'base_link'
        point.point.x = x
        point.point.y = y
        point.point.z = z
        point_pub.publish(point)
        rate.sleep()

if __name__ == '__main__':
    camera_coords = (0.176, -.464, 0.036)
    X_base, Y_base, Z_base = transform_coordinates(*camera_coords)
    
    if X_base is not None:
        rospy.loginfo(f"Transformed coordinates: X={X_base}, Y={Y_base}, Z={Z_base}")
        publish_point(X_base, Y_base, Z_base)
