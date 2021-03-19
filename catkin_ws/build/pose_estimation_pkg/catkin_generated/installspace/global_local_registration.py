#!/usr/bin/env python2

import rospy
import open3d as o3d
import numpy as np
from open3d_ros_helper import open3d_ros_helper as orh
import copy
import time
import sys
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

from functions import *

class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "camera_color_optical_frame"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "object_frame"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 1.0

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == "__main__":

    rospy.init_node('global_local_registration_tf2_broadcaster')

    start_time = time.time()
    print(":: Load two point clouds.")
    cad_model = o3d.io.read_point_cloud(sys.argv[1], print_progress=True)
    print("   Cad model has " + str(len(cad_model.points)) + " points")
    scene_pcd = o3d.io.read_point_cloud(sys.argv[2], print_progress=True)
    print("   Scene Point Cloud has " + str(len(scene_pcd.points)) + " points")

    ## Parameters for each component
    # default values
    z_distance = 0.5
    distance_threshold = 0.007
    voxel_size_component = 0.5

    if (sys.argv[3] == "m200"):
        z_distance = 0.49
        distance_threshold = 0.01
        voxel_size_component = 1

    if (sys.argv[3] == "m300"):
        z_distance = 0.5
        distance_threshold = 0.002
        voxel_size_component = 0.01

    # Threshold data with Z filter
    scene_pcd = threshold_filter_z(cloud=scene_pcd, z_distance=z_distance, display=False)
    # Segment the plane with RANSAC
    _, scene_pcd = segment_plane_ransac(cloud=scene_pcd, distance_threshold=distance_threshold, ransac_n=3, num_iterations=100, display=False)
    # DBSCAN Clustering
    scene_pcd_clustered, nb_clusters, scene_pcd = cluster_dbscan(cloud=scene_pcd, eps=0.01, min_samples=400, display=False)

    o3d.visualization.draw_geometries(scene_pcd_clustered)
    #o3d.visualization.draw_geometries([scene_pcd])
    print(">> Clustering time: %.3f" %(time.time() - start_time))

    for cluster in range(nb_clusters):
        print(">> Cluster %d: %d points" %(cluster, len(scene_pcd_clustered[cluster].points)))
    print("------------------------------------------------------------------------------\n")

    for i in range(1):
        
        print(":: Pose Estimation for Cluster %d" %(i))
        voxel_size = voxel_size_component
        start_time = time.time()
        source, target, source_down, target_down, source_fpfh, target_fpfh, source_down_fpfh, target_down_fpfh = prepare_dataset(source=cad_model,
                                                                                                                                target=scene_pcd_clustered[i], 
                                                                                                                                voxel_size = voxel_size)
        #draw_registration_result(source, target, np.identity(4))

        # Global Registration
        result_ransac, time_ransac = execute_global_registration(source=source_down,
                                                                target=target_down,
                                                                source_fpfh=source_down_fpfh,
                                                                target_fpfh=target_down_fpfh, 
                                                                voxel_size=voxel_size)
        print("   Global registration took %.3f sec." % (time_ransac))
        print(result_ransac)
        draw_registration_result(source, target, result_ransac.transformation)

        # The Global Registration worked better than the Fast Registration (the Fast Reg is faster)
        '''
        result_ransac, time_ransac = execute_fast_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
        print("Fast Global registration took %.3f sec." % (time_ransac))
        print(result_ransac)
        '''

        # Local Registration
        result_icp, time_icp = execute_local_registration(source=source, 
                                                        target=target,
                                                        source_fpfh=source_down_fpfh, 
                                                        target_fpfh=target_down_fpfh, 
                                                        result_ransac=result_ransac,
                                                        voxel_size=voxel_size)
        print("   Local registration took %.3f sec." % (time_icp))
        print(result_icp)
        print(result_icp.transformation)

        print(">> Registration Time form Cluster " + str(i) + ": %.3f" %(time.time() - start_time))
        print("------------------------------------------------------------------------------\n")

        draw_registration_result(source, target, result_icp.transformation)

        cad_pcd = copy.deepcopy(source_down)
        cad_pcd.transform(result_icp.transformation)
        bounding_box_cad = cad_pcd.get_oriented_bounding_box()
        bounding_box_cad.color = (1,0,0)

        dists = target_down.compute_point_cloud_distance(cad_pcd)
        dists = np.asarray(dists)
        ind = np.where(dists > 0.01)[0]
        target_down = target_down.select_down_sample(list(ind))
        scene_pcd_clustered[i] = target_down
        
        #o3d.visualization.draw_geometries([target_down, bounding_box_cad])

    #o3d.visualization.draw_geometries(scene_pcd_clustered)

    tfb = FixedTFBroadcaster()
    rospy.spin()
    print("DONE")