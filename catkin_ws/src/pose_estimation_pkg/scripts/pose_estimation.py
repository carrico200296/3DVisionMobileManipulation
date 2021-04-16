#! /usr/bin/env python

import rospy
import open3d as o3d
import numpy as np
import cv2
from open3d_ros_helper import open3d_ros_helper as orh
import copy
import time
import sys
import tf
import tf2_ros
import geometry_msgs.msg
import sensor_msgs

from functions import *


if __name__ == "__main__":

    scene_pcd = o3d.geometry.PointCloud()
    rospy.init_node('pose_estimation_tf2_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    print(":: Loading cad_model point cloud.")
    cad_model_pcd = o3d.io.read_point_cloud(sys.argv[1], print_progress=True)
    cad_model_pcd.translate(translation=(0, 0, 0), relative=False)
    cad_model_pcd = threshold_filter_circle(cloud=cad_model_pcd, radius=0.01, display=True)

    print(" :: Loading scene reconstructed point cloud.")
    #scene_pcd = o3d.io.read_point_cloud(sys.argv[2], print_progress=True)
    #data = rospy.wait_for_message("/camera/depth/color/points", sensor_msgs.msg.PointCloud2)
    data = rospy.wait_for_message("/pose_estimation/scene_reconstructed", sensor_msgs.msg.PointCloud2)
    scene_pcd = orh.rospc_to_o3dpc(data, remove_nans=True)
    print("   Cad model has %d points" %(len(cad_model_pcd.points)) )
    print("   Scene Point Cloud has %d points" %(len(scene_pcd.points)) )

    ## Parameters (default values)
    z_distance = 0.53
    x_distance = 0.2
    y_distance = 0.15
    distance_threshold = 0.00009
    voxel_size = 0.75 # works for m200

    start_preprocess_cluster = time.time()
    #scene_pcd = filter_pcd(scene_pcd, x_distance=x_distance, y_distance=y_distance, z_distance=z_distance)
    #_, scene_pcd = segment_plane_ransac(cloud=scene_pcd, distance_threshold=distance_threshold, ransac_n=3, num_iterations=100, display=False)
    scene_pcd = threshold_filter_min_max(scene_pcd, axis=0, min_distance=-0.15, max_distance=0.15)
    scene_pcd = threshold_filter_min_max(scene_pcd, axis=1, min_distance=-0.15, max_distance=0.15)
    scene_pcd = threshold_filter_min_max(scene_pcd, axis=2, min_distance=0.3, max_distance=0.53)
    #plane, scene_pcd = segment_plane_ransac(cloud=scene_pcd, distance_threshold=distance_threshold, ransac_n=3, num_iterations=100, display=False)
    scene_pcd, inliers = threshold_filter_color(cloud=scene_pcd, color_threshold=200)

    # DBSCAN Clustering
    # dbscan values for more density
    #scene_pcd_clustered, nb_clusters, scene_pcd = cluster_dbscan(cloud=scene_pcd, eps=0.01, min_samples=400, min_points_cluster = 4000, display=False)
    scene_pcd_clustered, nb_clusters, scene_pcd = cluster_dbscan(cloud=scene_pcd, eps=0.01, min_samples=100, min_points_cluster = 1000, display=False)
    end_preprocess_cluster = time.time()
    print(":: Final Scene Clusters to register:")
    for cluster in range(nb_clusters):
        nb_cluster_points = len(scene_pcd_clustered[cluster].points)
        print("   Cluster %d: %d points" %(cluster + 1, nb_cluster_points))

    print(":: Preprocessing and Clustering time: %.3f sec." %(end_preprocess_cluster - start_preprocess_cluster))
    print("------------------------------------------------------------------------------\n")
    o3d.visualization.draw_geometries(scene_pcd_clustered)

    # Global + Local Registration
    source, source_down, source_fpfh, source_down_fpfh = preprocess_source_pcd(source=cad_model_pcd, voxel_size=voxel_size)
    o3d.visualization.draw_geometries([source])
    o3d.visualization.draw_geometries([source_down])
    for i in range(nb_clusters):

        print("\n:: Calculating Pose Estimation for Cluster %d" %(i + 1))
        target, target_down, target_fpfh, target_down_fpfh = preprocess_point_cloud(cloud=scene_pcd_clustered[i], 
                                                                                    voxel_size = voxel_size)
        o3d.visualization.draw_geometries([target])
        o3d.visualization.draw_geometries([target_down])
        #draw_registration_result(source_down, target_down, np.identity(4))

        # Global Registration
        result_ransac, time_ransac = execute_global_registration(source=source_down,
                                                                target=target_down,
                                                                source_fpfh=source_down_fpfh,
                                                                target_fpfh=target_down_fpfh, 
                                                                voxel_size=voxel_size)
        #draw_registration_result(source, target, result_ransac.transformation)
        '''
        # The Global Registration worked better and slower than the Fast Registration
        result_ransac, time_ransac = execute_fast_global_registration(source_down,
                                                                    target_down,
                                                                    source_fpfh,
                                                                    target_fpfh,
                                                                    voxel_size)
        print("Fast Global Registration took %.3f sec." % (time_ransac))
        print(result_ransac)
        draw_registration_result(source, target, result_ransac.transformation)
        '''
        # Local Registration
        result_icp, time_icp = execute_local_registration(source=source, 
                                                        target=target,
                                                        result_ransac=result_ransac,
                                                        voxel_size=voxel_size)

        print(">> Registration TIME for Cluster %d : %.3f sec." %(i+1, time_ransac + time_icp))
        print("------------------------------------------------------------------------------\n")
        draw_registration_result(source, target, result_icp.transformation)

        cad_pcd = copy.deepcopy(source_down)
        cad_pcd.transform(result_icp.transformation)
        #mesh_cad_pcd = cad_pcd.compute_convex_hull()
        bounding_box_cad = cad_pcd.get_oriented_bounding_box()
        bounding_box_cad.color = (1,0,0)

        #correspondence_points = np.asarray(result_icp.correspondence_set)
        #print(correspondence_points.shape)

        dists = target_down.compute_point_cloud_distance(cad_pcd)
        dists = np.asarray(dists)
        ind = np.where(dists > 0.01)[0]
        target_down = target_down.select_down_sample(list(ind))
        scene_pcd_clustered[i] = target_down
        
        #o3d.visualization.draw_geometries([target_down, bounding_box_cad])
        frame_name = "object_frame_" + str(i)
        broadcaster_component_frame(broadcaster, frame_name, np.asarray(result_icp.transformation))

    #o3d.visualization.draw_geometries(scene_pcd_clustered)
    quit()
    rospy.spin()
    print("DONE")
