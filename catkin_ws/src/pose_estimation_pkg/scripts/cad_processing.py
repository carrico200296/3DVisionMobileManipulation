import open3d as o3d
import numpy as np
import copy
import sys
from scipy.spatial.transform import Rotation as R
from functions import *


def load_cad_model(path):
    cad_model_pcd = o3d.io.read_point_cloud(path, print_progress=True)
    cad_model_pcd = cad_model_pcd.scale(1.0/1000, center=True)
    cad_model_pcd = cad_model_pcd.voxel_down_sample(voxel_size=0.0007)
    cad_model_pcd.translate(translation=(0, 0, 0), relative=False)
    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.02)
    R = origin_frame.get_rotation_matrix_from_xyz((np.pi / 2, 0, 0))
    cad_model_pcd = threshold_filter_circle(cloud=cad_model_pcd, radius=0.009, display=False)
    cad_model_pcd.rotate(R, center=False)

    return cad_model_pcd, origin_frame

if __name__ == "__main__":

    cad_file_path = "/home/carlos/git/3DVisionMobileManipulation/catkin_ws/src/pose_estimation_pkg/data/m200.pcd"
    cad_model_pcd, origin_frame = load_cad_model(path=cad_file_path)
    o3d.visualization.draw_geometries([cad_model_pcd.paint_uniform_color([0.5, 0.5, 0.5])])
    picking_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    frames = [cad_model_pcd.paint_uniform_color([0.7, 0.7, 0.7])]
    for i in range(3):
        R = picking_frame.get_rotation_matrix_from_xyz((0, i*1.0472, 0))
        new_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        frames.append(new_frame.rotate(R, center=False))

    o3d.visualization.draw_geometries(frames)

    distances = cad_model_pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    print("   Cad model has %d points and avg_dist = %.6f" %(len(cad_model_pcd.points), avg_dist))