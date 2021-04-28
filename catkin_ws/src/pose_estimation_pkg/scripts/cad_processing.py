import open3d as o3d
import numpy as np

from functions import *

#cad_model_pcd = o3d.io.read_point_cloud(sys.argv[1], print_progress=True)
cad_file_path = "/home/carlos/git/3DVisionMobileManipulation/catkin_ws/src/pose_estimation_pkg/data/scaled_downsampled_m200.pcd"
cad_model_pcd = o3d.io.read_point_cloud(cad_file_path, print_progress=True)
cad_model_pcd.translate(translation=(0, 0, 0), relative=False)

mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
R = mesh.get_rotation_matrix_from_xyz((-np.pi / 2, 0, 0))
#R = mesh.get_rotation_matrix_from_axis_angle((np.pi, 0, 0))

origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
#origin_frame.rotate(R, center=False)
cad_model_pcd = threshold_filter_circle(cloud=cad_model_pcd, radius=0.009, display=False)
cad_model_pcd.rotate(R, center=False)
o3d.visualization.draw_geometries([cad_model_pcd, origin_frame])
