import open3d as o3d
import numpy as np
import copy
import sys

filename = sys.argv[1]
pcd = o3d.io.read_point_cloud(filename)

pcd_scaled = copy.deepcopy(pcd)
#pcd.translate((20,0,0), relative=False)
pcd_scaled.scale(1.0/1000, center=(0,0,0))

# visualize different point clouds
o3d.visualization.draw_geometries([pcd_scaled, pcd])

o3d.io.write_point_cloud("scaled_" + filename, pcd_scaled)