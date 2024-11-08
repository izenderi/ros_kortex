#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2

def point_cloud_callback(msg):
    rospy.loginfo("Point cloud received!")
    
    # Convert the ROS PointCloud2 message to a list of 3D points
    pc = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    cloud_points = np.array(list(pc))
    
    # Create a Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud_points)
    
    # Save the point cloud to a PLY file
    o3d.io.write_point_cloud("tissue_paper_cloud4.ply", pcd)
    
    rospy.loginfo("Point cloud saved to tissue_paper_cloud4.ply")

def capture_point_cloud():
    rospy.init_node('point_cloud_capture', anonymous=True)

    # Subscribe to the registered point cloud topic
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, point_cloud_callback)

    rospy.loginfo("Waiting for point cloud data...")
    
    # Keep the node running to listen for point cloud messages
    rospy.spin()

if __name__ == '__main__':
    try:
        capture_point_cloud()
    except rospy.ROSInterruptException:
        pass
