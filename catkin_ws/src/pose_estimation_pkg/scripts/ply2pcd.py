import open3d as o3d
import numpy as np
import copy
import sys

name = sys.argv[1]
filename = name + ".ply"
pcd = o3d.io.read_point_cloud(filename)
o3d.io.write_point_cloud(name + ".pcd", pcd)

# visualize pcd
o3d.visualization.draw_geometries([pcd])