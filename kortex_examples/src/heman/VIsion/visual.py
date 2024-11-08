import open3d as o3d

def load_and_visualize_point_cloud(filename):
    # Load the point cloud from the PCD file
    point_cloud = o3d.io.read_point_cloud(filename)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([point_cloud])

if __name__ == '__main__':
    load_and_visualize_point_cloud("/home/vboxuser/catkin_workspace/src/ros_kortex/kortex_examples/src/point_cloud.pcd")
