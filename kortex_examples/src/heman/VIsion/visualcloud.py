import open3d as o3d

def visualize_ply(file_path):
    # Read the PLY file
    pcd = o3d.io.read_point_cloud(file_path)
    
    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd], window_name="Point Cloud Visualization")

if __name__ == '__main__':
    file_path = 'tissue_paper_cloud2.ply'  # Path to your PLY file
    visualize_ply(file_path)
