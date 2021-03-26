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

def broadcaster_component_frame(broadcaster, object_frame_name, t_matrix):

    rot_matrix = t_matrix[:3,:3]
    pose_mat = np.eye(4)
    pose_mat[:3, :3] = rot_matrix
    trans_vector = t_matrix[:3,3]
    q = tf.transformations.quaternion_from_matrix(pose_mat)

    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "camera_depth_optical_frame"
    static_transformStamped.child_frame_id = object_frame_name

    static_transformStamped.transform.translation.x = trans_vector[0]
    static_transformStamped.transform.translation.y = trans_vector[1]
    static_transformStamped.transform.translation.z = trans_vector[2]

    static_transformStamped.transform.rotation.x = q[0]
    static_transformStamped.transform.rotation.y = q[1]
    static_transformStamped.transform.rotation.z = q[2]
    static_transformStamped.transform.rotation.w = q[3]

    broadcaster.sendTransform(static_transformStamped)


if __name__ == "__main__":

    ### IDEA: rospy.wait_for_service() when the node receive a service it will perform the components pose_estimation
    scene_pcd = o3d.geometry.PointCloud()
    rospy.init_node('pose_estimation_tf2_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    print(":: Load cad_model and the scene point clouds.")
    cad_model = o3d.io.read_point_cloud(sys.argv[1], print_progress=True)
    cad_model_without_middle = threshold_filter_x_y(cloud=cad_model, distance=0.015/2, display=True)
    #cad_model_without_middle.translate(translation=(0, 0, 0), relative=False)
    
    print("   Waiting for a scene reconstructed point cloud.")
    data = rospy.wait_for_message("/camera/depth/color/points", sensor_msgs.msg.PointCloud2)
    #data = rospy.wait_for_message("/pose_estimation/scene_reconstructed", sensor_msgs.msg.PointCloud2)
    scene_pcd = orh.rospc_to_o3dpc(data, remove_nans=True)
    #scene_pcd.voxel_down_sample(voxel_size=0.5)

    print("   Cad model has " + str(len(cad_model_without_middle.points)) + " points")
    #scene_pcd = o3d.io.read_point_cloud(sys.argv[2], print_progress=True)
    print("   Scene Point Cloud has " + str(len(scene_pcd.points)) + " points")

    ## Parameters for each component
    # default values
    z_distance = 0.53
    x_distance = 0.2
    y_distance = 0.15
    distance_threshold = 0.0009
    voxel_size = 1.0

    if (sys.argv[3] == "m200"):
        voxel_size = 1.0

    if (sys.argv[3] == "m300"):
        voxel_size = 0.01

    start_preprocess_cluster = time.time()
    scene_pcd = threshold_filter_z(cloud=scene_pcd, z_distance=z_distance, display=False)
    scene_pcd = threshold_filter_in_axis(scene_pcd, distance=x_distance, axis=0 ,keep_points_outside_threshold=False)
    scene_pcd = threshold_filter_in_axis(scene_pcd, distance=y_distance, axis=1 ,keep_points_outside_threshold=False, display=True)
    _, scene_pcd = segment_plane_ransac(cloud=scene_pcd, distance_threshold=distance_threshold, ransac_n=3, num_iterations=100, display=False)
    
    # DBSCAN Clustering
    scene_pcd_clustered, nb_clusters, scene_pcd = cluster_dbscan(cloud=scene_pcd, eps=0.01, min_samples=400, min_points_cluster = 4000, display=False)
    end_preprocess_cluster = time.time()

    print(":: Final Scene Clusters to register:")
    for cluster in range(nb_clusters):
        nb_cluster_points = len(scene_pcd_clustered[cluster].points)
        print("   Cluster %d: %d points" %(cluster + 1, nb_cluster_points))

    print(":: Preprocessing and Clustering time: %.3f sec." %(end_preprocess_cluster - start_preprocess_cluster))
    print("------------------------------------------------------------------------------\n")

    o3d.visualization.draw_geometries(scene_pcd_clustered)
    source, source_down, source_fpfh, source_down_fpfh = preprocess_source_pcd(source=cad_model_without_middle, voxel_size=voxel_size)

    for i in range(nb_clusters):

        print("\n:: Calculating Pose Estimation for Cluster %d" %(i + 1))
        target, target_down, target_fpfh, target_down_fpfh = preprocess_point_cloud(cloud=scene_pcd_clustered[i], 
                                                                                    voxel_size = voxel_size)
        #draw_registration_result(source, target, np.identity(4))

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
                                                        source_fpfh=source_fpfh, 
                                                        target_fpfh=target_fpfh, 
                                                        result_ransac=result_ransac,
                                                        voxel_size=voxel_size)
        print("\n:: Transformation Matrix:")
        print(result_icp.transformation)

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
        broadcaster_component_frame(broadcaster,frame_name, np.asarray(result_icp.transformation))

    #o3d.visualization.draw_geometries(scene_pcd_clustered)
    quit()
    rospy.spin()
    print("DONE")
