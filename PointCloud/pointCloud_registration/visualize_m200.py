import open3d as o3d
import numpy as np
import copy

#pcd = o3d.io.read_point_cloud("m200.ply")
#o3d.io.write_point_cloud("m200.pcd", pcd)

m200_pcd = o3d.io.read_point_cloud("m200.pcd")
m200_pcd.translate((0,0,0), relative=False)

voxel_size = 0.5
m200_pcd_down = m200_pcd.voxel_down_sample(voxel_size)

# visualize different point clouds
o3d.visualization.draw_geometries([m200_pcd])
