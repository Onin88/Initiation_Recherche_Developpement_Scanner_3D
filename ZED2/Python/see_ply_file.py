import open3d as o3d
import sys

# Vérifie s'il y a au moins un argument
if len(sys.argv) > 1:
    premier_argument = sys.argv[1]
    print("Le premier argument est :", premier_argument)
else:
    print("1 arg is required")


# Load PLY file
ply_path = premier_argument + ".ply"
pcd = o3d.io.read_point_cloud(ply_path)

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])