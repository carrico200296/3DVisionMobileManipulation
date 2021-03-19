import open3d as o3d
import numpy as np
import copy
import sys

filename = sys.argv[1]
pcd = o3d.io.read_point_cloud(filename)

pcd_scaled = copy.deepcopy(pcd)
pcd_scaled.scale(1.0/1000, center=(0,0,0))

o3d.visualization.draw_geometries([pcd_scaled, pcd])

if (sys.argv[2] == "save"):
    o3d.io.write_point_cloud("scaled_" + filename, pcd_scaled)