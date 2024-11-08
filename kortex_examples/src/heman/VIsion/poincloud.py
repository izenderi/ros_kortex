import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d

def save_point_cloud(cloud, filename):
    # Convert ROS PointCloud2 message to Open3D point cloud
    points = []
    for data in pc2.read_points(cloud, skip_nans=True):
        points.append([data[0], data[1], data[2]])

    # Create Open3D point cloud
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(np.array(points))

    # Save to PCD file
    o3d.io.write_point_cloud(filename, o3d_cloud)

def point_cloud_callback(msg):
    rospy.loginfo("Received point cloud data")
    save_point_cloud(msg, "//home/vboxuser/catkin_workspace/src/ros_kortex/kortex_examples/src/point_cloud.pcd")
    rospy.loginfo("Point cloud data saved")

def main():
    rospy.init_node('save_point_cloud')
    rospy.Subscriber('/camera/depth/points', PointCloud2, point_cloud_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
