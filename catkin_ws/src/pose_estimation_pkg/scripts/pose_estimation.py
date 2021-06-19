#! /usr/bin/env python

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
import xlsxwriter


if __name__ == "__main__":

    file_time = str(int(time.time()))
    file_name = '/home/carlos/Desktop/samples_for_report/results/sample_data_' + file_time + '.xlsx' 
    workbook = xlsxwriter.Workbook(file_name)
    worksheet = workbook.add_worksheet()
    row = 0
    column = 0
    headers = ["Component", "Source PCD points", "Target PCD points", "Correspondence Set", "Global time", "Global fitness", "Global RMSE", "Local time", "Local fitness", "Local RMSE", "Total time"]
    for item in headers:
        worksheet.write(row, column, item)
        column+=1

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
    data = rospy.wait_for_message("/pose_estimation/scene_reconstructed", sensor_msgs.msg.PointCloud2, rospy.Duration(1000.0))
    scene_pcd = orh.rospc_to_o3dpc(data, remove_nans=True)

    print(":: Loading cad_model point cloud.")
    cad_file_path = "/home/carlos/git/3DVisionMobileManipulation/catkin_ws/src/pose_estimation_pkg/data/m200.ply"
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
    sample_data = []
    row = 1
    column = 0
    object_frames = []
    tf_components_frames = []
    for i in range(nb_clusters):
        sample_data.append(str(i+1))

        print("\n:: Calculating 6D Pose for component %d" %(i + 1))
        target, target_down, target_fpfh, target_down_fpfh = preprocess_point_cloud(cloud=scene_pcd_clustered[i], 
                                                                                    voxel_size = voxel_size)
        result_ransac, time_ransac = execute_global_registration(source=source, target=target,
                                                                source_fpfh=source_fpfh, target_fpfh=target_fpfh, 
                                                                voxel_size=voxel_size)
        draw_registration_result(source, target, result_ransac.transformation)

        sample_data.append(len(source.points))
        sample_data.append(len(target.points))
        sample_data.append(len(result_ransac.correspondence_set))
        sample_data.append(time_ransac)
        sample_data.append(result_ransac.fitness)
        sample_data.append(result_ransac.inlier_rmse)

        result_icp, time_icp = execute_local_registration(source=source, target=target,
                                                        result_ransac=result_ransac, voxel_size=voxel_size)
        registration_time = time_ransac + time_icp
        print(">> Registration TIME for component %d : %.3f sec." %(i+1, registration_time))
        print("------------------------------------------------------------------------------\n")
        draw_registration_result(source, target, result_icp.transformation)

        sample_data.append(time_icp)
        sample_data.append(result_icp.fitness)
        sample_data.append(result_icp.inlier_rmse)
        sample_data.append(registration_time)

        object_frame = "object_frame_" + str(i+1)
        object_frames.append(object_frame)
        ref_frame = "view0_frame"
        tf_components_frames.append(result_icp.transformation)
        broadcaster_component_frame(static_broadcaster, ref_frame, object_frame, np.asarray(result_icp.transformation))

        for item in sample_data:
            worksheet.write(row, column, item)
            column+=1
        row+=1
        column = 0
        sample_data = []
    try: 
        print("Publishing components TF frames...")
        thread.start_new_thread(publish_components_pose, (static_broadcaster, object_frames, tf_components_frames))
    except:
        print("ERROR")

    print("6D Pose Estimation Module finished!")
    row+=1
    headers = ["Component", "X", "Y", "Z", "Rx", "Ry", "Rz", "y1", "y2", "y3"]
    for item in headers:
        worksheet.write(row, column, item)
        column+=1

    for i in range(len(object_frames)):
        time.sleep(1.0)
        pub_nbcomponents.publish(str(nb_clusters))

    base_frame = "base"
    list_transform_base_component = []
    for i in range(nb_clusters):
        component_frame = "object_frame_" + str(i+1)
        list_transform_base_component.append(tfBuffer.lookup_transform(base_frame, component_frame, rospy.Time(0)))

    pose_data = []
    column = 0
    row+=1
    for i in range(int(nb_clusters)):
        pose_data.append(str(i+1))

        tf_transform_base_component = list_transform_base_component[i]
        t, rot_vector, y_axis_original = get_t_rotvector_component(tf_transform_base_component=tf_transform_base_component)
        
        pose_data.append(t[0])
        pose_data.append(t[1])
        pose_data.append(t[2])
        pose_data.append(rot_vector[0])
        pose_data.append(rot_vector[1])
        pose_data.append(rot_vector[2])
        pose_data.append(y_axis_original[0])
        pose_data.append(y_axis_original[1])
        pose_data.append(y_axis_original[2])

        for item in pose_data:
            worksheet.write(row, column, item)
            column+=1
        row+=1
        column = 0
        pose_data = []

    workbook.close()

    rospy.spin()
    quit()
