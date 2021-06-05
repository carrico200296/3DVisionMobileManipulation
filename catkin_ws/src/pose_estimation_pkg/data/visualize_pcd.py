import open3d as o3d
import copy
import sys

filename = sys.argv[1]
pcd = o3d.io.read_point_cloud(filename)
frame_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
o3d.visualization.draw_geometries([pcd])