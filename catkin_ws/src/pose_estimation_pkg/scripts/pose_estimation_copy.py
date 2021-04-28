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

import thread
import rtde_control
import rtde_receive

def move_arm(pose, object_frame, tfBuffer, rtde_c):
    tf_transform_base_object = geometry_msgs.msg.TransformStamped()
    base_frame = "base"
    tf_transform_base_object = tfBuffer.lookup_transform(base_frame, object_frame, rospy.Time(), rospy.Duration(0.5))
    print(tf_transform_base_object.transform.translation)
    print(tf_transform_base_object.transform.rotation)
    rtde_c.moveL(pose, 0.05, 0.1)
    rtde_c.stopScript()

def publish_components_pose(broadcaster, pose_list):

    while not rospy.is_shutdown():

        for i, t_matrix in enumerate(pose_list):
            rot_matrix = t_matrix[:3,:3]
            pose_mat = np.eye(4)
            pose_mat[:3, :3] = rot_matrix
            trans_vector = t_matrix[:3,3]
            q = tf.transformations.quaternion_from_matrix(pose_mat)

            transformStamped = geometry_msgs.msg.TransformStamped()
            transformStamped.header.stamp = rospy.Time.now()
            #transformStamped.header.frame_id = "camera_depth_optical_frame"
            transformStamped.header.frame_id = "view0_frame"
            object_frame_name = "object_frame_" + str(i)
            transformStamped.child_frame_id = object_frame_name

            transformStamped.transform.translation.x = trans_vector[0]
            transformStamped.transform.translation.y = trans_vector[1]
            transformStamped.transform.translation.z = trans_vector[2]

            transformStamped.transform.rotation.x = q[0]
            transformStamped.transform.rotation.y = q[1]
            transformStamped.transform.rotation.z = q[2]
            transformStamped.transform.rotation.w = q[3]

            broadcaster.sendTransform(transformStamped)


if __name__ == "__main__":

    ## Parameters (default values)
    distance_threshold_plane = 0.004
    voxel_size = 1.0 # works for m200 and no-downsampled pointcoulds

    scene_pcd = o3d.geometry.PointCloud()
    rospy.init_node('pose_estimation_node')
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    print(":: Loading cad_model point cloud.")
    #cad_model_pcd = o3d.io.read_point_cloud(sys.argv[1], print_progress=True)
    cad_file_path = "/home/carlos/git/3DVisionMobileManipulation/catkin_ws/src/pose_estimation_pkg/data/scaled_downsampled_m200.pcd"
    cad_model_pcd = o3d.io.read_point_cloud(cad_file_path, print_progress=True)
    cad_model_pcd.translate(translation=(0, 0, 0), relative=False)
    cad_model_pcd = threshold_filter_circle(cloud=cad_model_pcd, radius=0.009, display=False)
    source, source_down, source_fpfh, source_down_fpfh = preprocess_source_pcd(source=cad_model_pcd, voxel_size=voxel_size)

    print(" :: Loading scene reconstructed point cloud.")
    #scene_pcd = o3d.io.read_point_cloud(sys.argv[2], print_progress=True)
    data = rospy.wait_for_message("/pose_estimation/scene_reconstructed", sensor_msgs.msg.PointCloud2, rospy.Duration(60.0))
    scene_pcd = orh.rospc_to_o3dpc(data, remove_nans=True)
    scene_pcd, inliers = threshold_filter_color(cloud=scene_pcd, color_threshold=150)
    o3d.visualization.draw_geometries([scene_pcd])

    print("   Cad model has %d points" %(len(cad_model_pcd.points)) )
    print("   Scene Point Cloud has %d points" %(len(scene_pcd.points)) )

    # DBSCAN Clustering
    start_preprocess_cluster = time.time()
    scene_pcd_clustered, nb_clusters, scene_pcd = cluster_dbscan(cloud=scene_pcd, eps=0.01, min_samples=100, min_points_cluster = 2000, display=False)
    end_preprocess_cluster = time.time()
    print(":: Preprocessing and Clustering time: %.3f sec." %(end_preprocess_cluster - start_preprocess_cluster))
    print("------------------------------------------------------------------------------\n")
    #o3d.visualization.draw_geometries(scene_pcd_clustered)

    # Global + Local Registration
    #o3d.visualization.draw_geometries([source])
    #o3d.visualization.draw_geometries([source_down])
    pose1 = [0.43824456892579444, 0.24227909620285548, 0.46318420864958404, 0.03370084992098882, 3.0669104890781944, -0.44672143038415657]
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    tf_components_frames = []
    for i in range(nb_clusters):

        print("\n:: Calculating Pose Estimation for Cluster %d" %(i + 1))
        target, target_down, target_fpfh, target_down_fpfh = preprocess_point_cloud(cloud=scene_pcd_clustered[i], 
                                                                                    voxel_size = voxel_size)
        #o3d.visualization.draw_geometries([target])
        #o3d.visualization.draw_geometries([target_down])
        #draw_registration_result(source_down, target_down, np.identity(4))

        result_ransac, time_ransac = execute_global_registration(source=source,
                                                                target=target,
                                                                source_fpfh=source_fpfh,
                                                                target_fpfh=target_fpfh, 
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
        result_icp, time_icp = execute_local_registration(source=source, 
                                                        target=target,
                                                        result_ransac=result_ransac,
                                                        voxel_size=voxel_size)

        print(">> Registration TIME for Cluster %d : %.3f sec." %(i+1, time_ransac + time_icp))
        print("------------------------------------------------------------------------------\n")
        #draw_registration_result(source, target, result_icp.transformation)

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
        object_frame = "object_frame_" + str(i)
        ref_frame = "view0_frame"
        #ref_frame = "camera_depth_optical_frame"
        tf_components_frames.append(result_icp.transformation)
        broadcaster_component_frame(broadcaster, ref_frame, object_frame, np.asarray(result_icp.transformation))

        if i == 0:
            try: 
                print(":: Picking object 1")
                #rtde_c = rtde_control.RTDEControlInterface("192.168.10.20")
                #thread.start_new_thread(move_arm, (pose1, object_frame, tfBuffer, rtde_c))
            except:
                print("ERROR")

    print(tf_components_frames)
    #o3d.visualization.draw_geometries(scene_pcd_clustered)
    time.sleep(5.0)
    rospy.spin()
    quit()
    print("DONE")
