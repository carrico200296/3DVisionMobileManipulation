#! /usr/bin/env python

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from sklearn import cluster
import sklearn
import copy
import time

import rospy
from open3d_ros_helper import open3d_ros_helper as orh
import cv2
import sys
import tf
import tf2_ros
import geometry_msgs.msg
import sensor_msgs


# FUNCTIONS FOR FILTERING AND THRESHOLDING A POINTCLOUD ------------------------------------------------------------------

def display_inlier_outlier(cloud, ind):
    # Funtion to display the pointcloud with the outliers in red

    inlier_cloud = cloud.select_down_sample(ind)
    outlier_cloud = cloud.select_down_sample(ind, invert=True)
    outlier_cloud.paint_uniform_color([1, 0, 0])

    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

def remove_color_outlier(cloud, color_threshold=150):
    # Funtion to remove all the points with R,G and B color value < than a color_threshold

    print(":: Perform Threshold Color Filter")
    pcd_temp = copy.deepcopy(cloud)
    colors = np.asarray(pcd_temp.colors)
    threshold_mask0 = colors[:, 0]*255 < color_threshold
    threshold_mask1 = colors[:, 1]*255 < color_threshold
    threshold_mask2 = colors[:, 2]*255 < color_threshold
    threshold_mask01 = np.logical_and(threshold_mask0, threshold_mask1)
    threshold_mask = np.logical_and(threshold_mask01, threshold_mask2)

    inliers = []
    for i,item_mask in enumerate(threshold_mask):
        if item_mask==False:
            inliers.append(i)
    pcd = pcd_temp.select_down_sample(inliers)

    return pcd, inliers

def threshold_filter_in_axis(cloud, distance, axis, keep_points_outside_threshold = False, display=False):

    print(":: Perform Threshold Filter in Axis: %d" %axis)
    pcd_temp = copy.deepcopy(cloud)
    points = np.asarray(pcd_temp.points)
    threshold_mask = np.where(abs(points[:, axis]) < distance)[0]
    pcd = pcd_temp.select_down_sample(list(threshold_mask), invert=keep_points_outside_threshold)

    if (display == True):
        o3d.visualization.draw_geometries([pcd])

    return pcd

def threshold_filter_z(cloud, z_distance=0.50, display=False):

    print(":: Perform Threshold Z Filter")
    pcd_temp = copy.deepcopy(cloud)
    points = np.asarray(pcd_temp.points)
    threshold_mask = np.where(points[:, 2] < z_distance)[0]
    print("   Point Cloud after Threshold Z Filter = " + str(len(threshold_mask)) + " points")
    pcd = pcd_temp.select_down_sample(list(threshold_mask))

    if (display == True):
        o3d.visualization.draw_geometries([pcd])

    return pcd

def threshold_filter_min_max(cloud, axis, min_distance, max_distance, display=False):

    print(":: Perform Threshold Filter min_max distance in axis %d" %axis)
    pcd_temp = copy.deepcopy(cloud)

    points = np.asarray(pcd_temp.points)
    threshold_mask_max = np.where(points[:, axis] < max_distance)[0]
    pcd_temp = pcd_temp.select_down_sample(list(threshold_mask_max))

    points = np.asarray(pcd_temp.points)
    threshold_mask_min = np.where(points[:, axis] > min_distance)[0]
    pcd = pcd_temp.select_down_sample(list(threshold_mask_min))

    print("   Point Cloud after Threshold Filter = %d points" %(len(threshold_mask_min)))

    if (display == True):
        o3d.visualization.draw_geometries([pcd])

    return pcd

def filter_pcd(pcd, x_distance = 0.2, y_distance = 0.15, z_distance = 0.53):

    # These Parameters will be define by the ARUCO ROI detected
    pcd = threshold_filter_z(pcd, z_distance=z_distance, display=False)
    pcd = threshold_filter_in_axis(pcd, distance=x_distance, axis=0 ,keep_points_outside_threshold=False, display=False)
    pcd = threshold_filter_in_axis(pcd, distance=y_distance, axis=1 ,keep_points_outside_threshold=False, display=False)
    
    return pcd

def threshold_filter_circle(cloud, radius=0.01, display=False):

    print(":: Perform Threshold X and Y Filter")
    pcd_temp = copy.deepcopy(cloud)
    points = np.asarray(pcd_temp.points)
    threshold_mask = np.where( ( pow(points[:, 0],2) + pow(points[:,1],2) ) <= pow(radius, 2) )[0]
    pcd = pcd_temp.select_down_sample(list(threshold_mask), invert=True)

    if (display == True):
        frame_source = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=pcd.get_center())
        frame_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        o3d.visualization.draw_geometries([pcd, frame_source, frame_origin])

    return pcd



# FUNCTIONS FOR RANSAC PLANE SEGMENTATION AND DBSCAN CLUSTERING OBJECTS --------------------------------------------------

def segment_plane_ransac(cloud, distance_threshold=0.007, ransac_n=3, num_iterations=100, display=False):

    print(":: Perform RANSAC Plane Segmentation")
    pcd_temp = copy.deepcopy(cloud)
    plane_model, inliers_plane = pcd_temp.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=100)
    [a, b, c, d] = plane_model
    print("   Plane Equation %.2fx + %.2fy + %.2fz + %.2f = 0" % (a, b, c, d))
    plane_cloud, pcd = remove_outliers(cloud=pcd_temp, outliers=inliers_plane, display=display)
    print("   Point Cloud after RANSAC plane segmentation = " + str(len(pcd.points)) + " points")

    return plane_cloud, pcd

def remove_outliers(cloud, outliers, display=False):
    # INPUTS:
    # - cloud:: open3d.geometry.PointCloud
    # - outlier:: List[int] of outliers to display

    outlier_cloud = cloud.select_down_sample(outliers)
    pcd = cloud.select_down_sample(outliers, invert=True)
    outlier_cloud.paint_uniform_color([1, 0, 0])

    if (display == True):
        o3d.visualization.draw_geometries([outlier_cloud, pcd])

    return outlier_cloud, pcd

def cluster_dbscan(cloud, eps=0.01, min_samples=400, min_points_cluster=4000, display=False):

    print(":: DBSCAN Clustering")
    pcd_temp = copy.deepcopy(cloud)
    xyz = np.asarray(pcd_temp.points)
    clustering_dbsan = sklearn.cluster.DBSCAN(eps=eps, min_samples=min_samples).fit(xyz)
    labels_dbscan = clustering_dbsan.labels_

    pcd_clustered, nb_clusters, pcd = extract_clusters(cloud=pcd_temp, labels=labels_dbscan, display=display)

    final_clusters = []
    final_clusters = list(pcd_clustered)
    for cluster in range(nb_clusters):
        nb_cluster_points = len(pcd_clustered[cluster].points)
        if nb_cluster_points < min_points_cluster:
            del final_clusters[cluster]
            nb_clusters = nb_clusters - 1
            print("   Cluster %d removed. Points: %d" % (cluster + 1, nb_cluster_points))

    return final_clusters, nb_clusters, pcd

def extract_clusters(cloud, labels, display=False):

    pcd_temp = copy.deepcopy(cloud)
    nb_clusters = labels.max() + 1
    print("   Point cloud has %d clusters" % (nb_clusters))

    cmap = plt.get_cmap("tab20")
    colors = cmap(labels / (float(nb_clusters) if nb_clusters > 0 else 1.0))
    colors[labels < 0] = 0
    pcd_temp.colors = o3d.utility.Vector3dVector(colors[:, :3])

    pcd = cloud.select_down_sample(list(np.where(labels > -1)[0]))
    pcd_clustered = []

    for i in range(nb_clusters):
        pcd_clustered.append(pcd_temp.select_down_sample(list(np.where(labels == i)[0])))

    if (display == True):
        o3d.visualization.draw_geometries(pcd_clustered)

    return pcd_clustered, nb_clusters, pcd



# FUNCTIONS FOR PREPROCESSING A POINTCLOUD ------------------------------------------------------------------------------

def scale_point_cloud(cloud, scale_factor):

    print(":: Scaling Point Cloud with factor %.2f" % (scale_factor))
    cloud.scale(scale=scale_factor, center=True)

    return cloud

def downsample_point_cloud(cloud, voxel_size):

    print(":: Downsampling Point Cloud with voxel size = %.2f" %(voxel_size))
    pcd = cloud.voxel_down_sample(voxel_size=voxel_size)

    return pcd

def preprocess_source_pcd(source, voxel_size, scale_factor = 1.0/1000):
    
    print(":: Preprocess CAD point cloud")
    #print("   Scaling with scale_factor %.4f." % scale_factor)
    # Scale and downsample is already done when we load the model
    #source.scale(scale=scale_factor, center=True)

    source = copy.deepcopy(source)
    source_down = copy.deepcopy(source)
    distances = source_down.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    voxel_down = avg_dist * 1.2
    print("   Downsampling with voxel_down %.3f." % voxel_down)
    source_down = source_down.voxel_down_sample(voxel_down)

    radius_normal = voxel_size * 2
    print("   Estimate normals with search radius %.3f." % radius_normal)
    source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=50))
    source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=50))

    radius_feature = voxel_size * 5
    print("   Compute FPFH feature with search radius %.3f." % radius_feature)
    source_fpfh = o3d.registration.compute_fpfh_feature(source, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    source_down_fpfh = o3d.registration.compute_fpfh_feature(source_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    return source, source_down, source_fpfh, source_down_fpfh

def preprocess_point_cloud(cloud, voxel_size):

    print(":: Preprocess target point cloud")
    pcd = copy.deepcopy(cloud)
    pcd_down = copy.deepcopy(cloud)
    distances = pcd_down.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    voxel_down = avg_dist * 2.5
    print("   Downsample with a voxel_down size %.3f." % voxel_down)
    pcd_down = pcd_down.voxel_down_sample(voxel_down)

    radius_normal = voxel_size * 2
    print("   Estimate normals with search radius %.3f." % radius_normal)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=50))
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=50))
    
    radius_feature = voxel_size * 5
    print("   Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    pcd_down_fpfh = o3d.registration.compute_fpfh_feature(pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    return pcd, pcd_down, pcd_fpfh, pcd_down_fpfh



# FUNCTIONS FOR GLOBAL AND LOCAL REGISTRATION ----------------------------------------------------------------------------

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # Card model in gray
    source_temp.paint_uniform_color([0.7, 0.7, 0.7])

    source_temp.transform(transformation)
    frame_source = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    frame_source.transform(transformation)
    frame_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    o3d.visualization.draw_geometries([source_temp, target_temp, frame_source, frame_origin])

def execute_global_registration(source, target, source_fpfh, target_fpfh,
                                voxel_size):

    max_correspondence_distance = 0.1 #voxel_size * 0.5 # before was 1.5
    estimation_method = o3d.registration.TransformationEstimationPointToPoint(False)
    #estimation_method = o3d.registration.TransformationEstimationPointToPlane() PointToPlane is not working!!!
    ransac_n = 4
    edge_length = 0.9
    checkers = [o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(edge_length),
                o3d.registration.CorrespondenceCheckerBasedOnDistance(max_correspondence_distance)]
    #           CorrespondenceCheckerBasedOnNormal
    max_iteration = 400000 #0 # with 100000 didnt work well, with 4000000 takes too much time but works
    max_validation = 500
    criteria = o3d.registration.RANSACConvergenceCriteria(max_iteration, max_validation)

    print("\n:: Global Alignment: RANSAC Registration.")
    print("   max_correnpondence_distance = %.3f." % max_correspondence_distance)
    print("   estimation_method = PointToPoint")
    print("   ransac_n = %d" % ransac_n)
    print("   CorrespondenceCheckerBasedOnEdgeLength = %.3f" % edge_length)
    print("   CorrespondenceCheckerBasedOnDistance = %.3f" % max_correspondence_distance)
    print("   RANSACConvergenceCriteria: max_iteration = %d  max_validation = %d" % (max_iteration, max_validation))

    start_global = time.time()
    result = o3d.registration.registration_ransac_based_on_feature_matching(source, target, source_fpfh, target_fpfh,
                                                                               max_correspondence_distance,
                                                                               estimation_method, ransac_n,
                                                                               checkers, criteria)
    time_global = time.time() - start_global
    print(result)
    print("   Global Registration TIME: %.3f sec." % (time_global))

    return result, time_global

def execute_fast_global_registration(source, target, source_fpfh,
                                     target_fpfh, voxel_size):

    max_correspondence_distance = voxel_size * 0.5
    FastGlobalRegistrationOption = o3d.registration.FastGlobalRegistrationOption(division_factor=voxel_size,
                                                                                maximum_correspondence_distance=max_correspondence_distance,
                                                                                iteration_number=5000)

    print("\n:: Global Aligment: Fast Global Registration")
    print("   max_correspondence_distance = %.3f" % max_correspondence_distance)

    start_fast_global = time.time()
    result = o3d.registration.registration_fast_based_on_feature_matching(source, target, source_fpfh, target_fpfh,
                                                                          FastGlobalRegistrationOption)
    time_fast_global = time.time() - start_fast_global

    return result, time_fast_global

def execute_local_registration(source, target, result_ransac, voxel_size):

    start_local = time.time()
    max_correspondence_distance = 0.2 #voxel_size * 0.4 # before was 0.4
    init_trans = result_ransac.transformation
    #estimation_method = o3d.registration.TransformationEstimationPointToPoint(False)
    estimation_method = o3d.registration.TransformationEstimationPointToPlane()
    relative_fitness = 1e-9 # with 1e-10 too much time
    relative_rmse = 1e-7 # with 1e-8 too much time
    max_iteration = 500 
    criteria = o3d.registration.ICPConvergenceCriteria(
                                            relative_fitness=relative_fitness,
                                            relative_rmse=relative_rmse,
                                            max_iteration=max_iteration)

    print("\n:: Local Alignment: Point-to-Plane ICP registration")
    print("   max_correnpondence_distance = %.3f." % max_correspondence_distance)
    print("   estimation_method = PointToPlane")
    print("   ICPConvergenceCriteria: relative_fitness = 1e-10, relative_rmse = 1e-8, max_iteration = %d" % (max_iteration))

    result = o3d.registration.registration_icp(source, target, max_correspondence_distance, 
                                              init_trans,
                                              estimation_method,
                                              criteria)
    time_local = time.time() - start_local

    print(result)
    print("   Local Registration TIME: %.3f sec." % (time_local))

    return result, time_local



# FUNCTIONS FOR TF TRANSFORMATIONS

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

def broadcaster_scene_view_frame(broadcaster, scene_view_frame, transformation):

    transformation.header.stamp = rospy.Time.now()
    transformation.child_frame_id = scene_view_frame
    broadcaster.sendTransform(transformation)

def tf_transform_pcd_ICP(source_pcd, target_pcd, tf_transform, pcd_view):
    # source_pcd = the pcd that has to be move (view2,3,4)
    # target_pcd = the pcd that actuates as static (view0)
    # tf_transform = is a tf_transformation from a view(source_pcd) to the view0(target_pcd)

    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = tf_transform.transform.rotation.x
    q[1] = tf_transform.transform.rotation.y
    q[2] = tf_transform.transform.rotation.z
    q[3] = tf_transform.transform.rotation.w
    rot_mat = tf.transformations.quaternion_matrix(q)
    trans = [0.0, 0.0, 0.0]
    trans[0] = tf_transform.transform.translation.x
    trans[1] = tf_transform.transform.translation.y
    trans[2] = tf_transform.transform.translation.z

    T = np.eye(4)
    T[:3, :3] = rot_mat[:3, :3]
    T[:3,3] = trans

    pcd_view.transform(T)
    source_pcd_IPC = copy.deepcopy(source_pcd)
    source_pcd_IPC.transform(T)
    #source_pcd_IPC.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=30))
    #source_pcd_IPC.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=30))

    estimation_method = o3d.registration.TransformationEstimationPointToPoint()
    relative_fitness = 1e-6  # 1e-9 too much time
    relative_rmse = 1e-6 # 1e-7 too much time
    max_iteration = 100 # 500 too much
    criteria = o3d.registration.ICPConvergenceCriteria(
                                            relative_fitness=relative_fitness,
                                            relative_rmse=relative_rmse,
                                            max_iteration=max_iteration)
    print(":: Performing ICP with the view0")
    icp_result = o3d.registration.registration_icp(source_pcd_IPC, target_pcd, max_correspondence_distance=0.009,
                                                   estimation_method=estimation_method, criteria=criteria)
    source_pcd_IPC = source_pcd_IPC.transform(icp_result.transformation)

    return source_pcd_IPC, pcd_view

def get_pcd_roi(cloud, pose_nb, tfBuffer):
    scene_pcd = copy.deepcopy(cloud)
    detected_roi = []
    size_aruco = 0.04
    for i in range(4):
        aruco_id_frame = "depth_aruco_id_" + str(i+1)
        tf_transform = tfBuffer.lookup_transform("camera_depth_optical_frame", aruco_id_frame, rospy.Time(0), rospy.Duration(1.5))
        detected_roi.append((tf_transform.transform.translation.x,
                             tf_transform.transform.translation.y,
                             tf_transform.transform.translation.z))
    print(pose_nb)
    if pose_nb == 0:
        x_min = detected_roi[0][0] - size_aruco 
        x_max = detected_roi[1][0] + size_aruco
        y_min = detected_roi[0][1] - size_aruco
        y_max = detected_roi[2][1] + size_aruco
        z_max = detected_roi[0][2]
        z_min = z_max - 0.2

    if pose_nb == 1:
        x_min = detected_roi[1][0] - size_aruco 
        x_max = detected_roi[3][0] + size_aruco
        y_min = detected_roi[1][1] - size_aruco
        y_max = detected_roi[0][1] + size_aruco
        z_max = detected_roi[1][2]
        z_min = z_max - 0.2

    if pose_nb == 2:
        x_min = detected_roi[3][0] - size_aruco 
        x_max = detected_roi[2][0] + size_aruco
        y_min = detected_roi[3][1] - size_aruco
        y_max = detected_roi[1][1] + size_aruco
        z_max = detected_roi[3][2]
        z_min = z_max - 0.2

    if pose_nb == 3:
        x_min = detected_roi[2][0] - size_aruco 
        x_max = detected_roi[0][0] + size_aruco
        y_min = detected_roi[2][1] - size_aruco
        y_max = detected_roi[3][1] + size_aruco
        z_max = detected_roi[2][2]
        z_min = z_max - 0.2

    print("x_min = %.3f" %x_min)
    print("x_max = %.3f" %x_max)
    print("y_min = %.3f" %y_min)
    print("y_max = %.3f" %y_max)
    print("z_min = %.3f" %z_min)
    print("z_max = %.3f" %z_max)

    scene_pcd = threshold_filter_min_max(scene_pcd, axis=0, min_distance=x_min, max_distance=x_max)
    scene_pcd = threshold_filter_min_max(scene_pcd, axis=1, min_distance=y_min, max_distance=y_max)
    scene_pcd = threshold_filter_min_max(scene_pcd, axis=2, min_distance=z_min, max_distance=z_max)

    return scene_pcd

