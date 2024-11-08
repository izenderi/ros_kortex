import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_marker(x, y, z, frame_id='camera_color_frame'):
    rospy.init_node('dimension_visualizer', anonymous=True)
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points_and_lines"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # Adjust size as needed
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0  # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    # Example coordinates in meters
    x, y, z = 0.20, 0.53, 0.50
    publish_marker(x, y, z)
