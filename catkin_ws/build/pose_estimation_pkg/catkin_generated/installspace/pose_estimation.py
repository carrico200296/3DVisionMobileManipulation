#!/usr/bin/env python2

import rospy
import open3d as o3d
import numpy as np
import cv2
from open3d_ros_helper import open3d_ros_helper as orh
from scipy.spatial.transform import Rotation as R
import copy
import time
import sys
import tf
import tf2_ros
import geometry_msgs.msg
import sensor_msgs
import std_msgs

from functions import *

import thread
import rtde_control
import rtde_receive

def publish_components_pose(broadcaster, object_frames, tf_components_frames):

    while not rospy.is_shutdown():
        for i in range(len(tf_components_frames)):
            ref_frame = "view0_frame"
            transformation = tf_components_frames[i]
            broadcaster_component_frame(broadcaster, ref_frame, object_frames[i], np.asarray(transformation))
            #transformStamped_component = from_Tmatrix_to_tf(ref_frame, object_frames[i], np.asarray(transformation))
        time.sleep(0.5)

def load_cad_model(path):
    cad_model_pcd = o3d.io.read_point_cloud(path, print_progress=True)
    #cad_model_pcd = cad_model_pcd.scale(1.0/1000, center=True)
    #cad_model_pcd = cad_model_pcd.voxel_down_sample(voxel_size=0.0009) # with voxel_size = 0.002 is working always (but up-down problem)
    cad_model_pcd.translate(translation=(0, 0, 0), relative=False)
    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    R = origin_frame.get_rotation_matrix_from_xyz((np.pi / 2, 0, 0))
    cad_model_pcd = threshold_filter_circle(cloud=cad_model_pcd, radius=0.009, display=False)
    cad_model_pcd.rotate(R, center=False)

    return cad_model_pcd, origin_frame


if __name__ == "__main__":
    
    rospy.init_node('pose_estimation_node')
    pub_nbcomponents = rospy.Publisher("/pose_estimation/nb_components", std_msgs.msg.String, queue_size=1)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    ## Parameters
    distance_threshold_plane = 0.004
    voxel_size = 1.0 # works for m200 and no-downsampled pointcoulds

    print(" :: Waiting for the scene reconstructed point cloud.")
    data = rospy.wait_for_message("/pose_estimation/scene_reconstructed", sensor_msgs.msg.PointCloud2, rospy.Duration(700.0))
    scene_pcd = orh.rospc_to_o3dpc(data, remove_nans=True)

    print(":: Loading cad_model point cloud.")
    cad_file_path = "/home/carlos/git/3DVisionMobileManipulation/catkin_ws/src/pose_estimation_pkg/data/scaled_downsampled_m200.pcd"
    cad_model_pcd, origin_frame = load_cad_model(path=cad_file_path)
    #o3d.visualization.draw_geometries([cad_model_pcd, origin_frame])
    source, source_down, source_fpfh, source_down_fpfh = preprocess_source_pcd(source=cad_model_pcd, voxel_size=voxel_size)

    distances = source.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    print(":: Cad model has %d points and avg_dist = %.6f" %(len(source.points), avg_dist))
    distances = scene_pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    print(":: Scene Point Cloud has %d points and avg_dist = %.6f" %(len(scene_pcd.points), avg_dist))

    # DBSCAN Clustering
    start_dbscan = time.time()
    # instead of eps=0.01 (8 seconds), using eps=0.004 (3 seconds), needs to be check with some samples
    scene_pcd_clustered, nb_clusters, scene_pcd = cluster_dbscan(cloud=scene_pcd, eps=0.004, min_samples=100, min_points_cluster = 2000, display=False)
    end_dbscan = time.time()
    print(":: Clustering time: %.3f sec." %(end_dbscan - start_dbscan))
    print("------------------------------------------------------------------------------\n")
    o3d.visualization.draw_geometries(scene_pcd_clustered)

    # Global + Local Registration
    object_frames = []
    tf_components_frames = []
    for i in range(nb_clusters):

        print("\n:: Calculating 6D Pose for component %d" %(i + 1))
        target, target_down, target_fpfh, target_down_fpfh = preprocess_point_cloud(cloud=scene_pcd_clustered[i], 
                                                                                    voxel_size = voxel_size)
        result_ransac, time_ransac = execute_global_registration(source=source, target=target,
                                                                source_fpfh=source_fpfh, target_fpfh=target_fpfh, 
                                                                voxel_size=voxel_size)
        draw_registration_result(source, target, result_ransac.transformation)
        result_icp, time_icp = execute_local_registration(source=source, target=target,
                                                        result_ransac=result_ransac, voxel_size=voxel_size)
        print(">> Registration TIME for component %d : %.3f sec." %(i+1, time_ransac + time_icp))
        print("------------------------------------------------------------------------------\n")
        draw_registration_result(source, target, result_icp.transformation)

        object_frame = "object_frame_" + str(i+1)
        object_frames.append(object_frame)
        ref_frame = "view0_frame"
        tf_components_frames.append(result_icp.transformation)
        broadcaster_component_frame(static_broadcaster, ref_frame, object_frame, np.asarray(result_icp.transformation))

    try: 
        print("Publishing components TF frames...")
        thread.start_new_thread(publish_components_pose, (static_broadcaster, object_frames, tf_components_frames))
    except:
        print("ERROR")

    print("6D Pose Estimation Module finished!")

    base_frame = "base"
    for i in range(len(object_frames)):
        time.sleep(1.0)
        pub_nbcomponents.publish(str(nb_clusters))

    rospy.spin()
    quit()
