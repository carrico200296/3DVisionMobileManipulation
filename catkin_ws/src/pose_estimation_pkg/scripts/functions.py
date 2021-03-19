#! /usr/bin/env python

import numpy as np
import open3d.open3d as o3d
import matplotlib.pyplot as plt
from sklearn import cluster
import sklearn
import copy
import time

def threshold_filter_x_y(cloud, distance=0.01, display=False):

    print(":: Perform Threshold X Filter")
    pcd_temp = copy.deepcopy(cloud)
    points = np.asarray(pcd_temp.points)
    threshold_mask_x = np.where(abs(points[:, 0]) > distance)[0]
    threshold_mask_y = np.where(abs(points[:, 1]) > distance)[0]
    print("   Point Cloud after Threshold X Filter = %d points" % (len(threshold_mask_x) + len(threshold_mask_y)))
    pcd_x = pcd_temp.select_down_sample(list(threshold_mask_x))
    pcd_y = pcd_temp.select_down_sample(list(threshold_mask_y))
    pcd = pcd_x + pcd_y

    if (display == True):
        frame_source = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=pcd.get_center())
        frame_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        o3d.visualization.draw_geometries([pcd, frame_source, frame_origin])

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

def segment_plane_ransac(cloud, distance_threshold=0.007, ransac_n=3, num_iterations=100, display=False):

    print(":: Perform RANSAC Plane Segmentation")
    pcd_temp = copy.deepcopy(cloud)
    plane_model, inliers_plane = pcd_temp.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=100)
    [a, b, c, d] = plane_model
    print("   Plane Equation %.2fx + %.2fy + %.2fz + %.2f = 0" % (a, b, c, d))
    plane_cloud, pcd = remove_outliers(cloud=pcd_temp, outliers=inliers_plane, display=display)
    print("   Point Cloud after RANSAC plane segmentation = " + str(len(pcd.points)) + " points")

    return plane_cloud, pcd

def cluster_dbscan(cloud, eps=0.01, min_samples=300, min_points_cluster=3000, display=False):

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
        if (nb_cluster_points < min_points_cluster):
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


def scale_point_cloud(cloud, scale_factor):

    print(":: Scaling Point Cloud with factor %.2f" % (scale_factor))
    cloud.scale(scale=scale_factor, center=True)

    return cloud


def downsample_point_cloud(cloud, voxel_size):

    print(":: Downsampling Point Cloud with voxel size = %.2f" %(voxel_size))
    pcd = cloud.voxel_down_sample(voxel_size=voxel_size)

    return pcd

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # print(f'Center of source before transformation: {source_temp.get_center()}')
    # print(f'Center of target before transformation: {target_temp.get_center()}')
    # Card model in yellow
    #source_temp.paint_uniform_color([1, 0.706, 0])
    source_temp.paint_uniform_color([0.7, 0.7, 0.7])

    source_temp.transform(transformation)
    frame_source = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=source_temp.get_center())
    frame_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    o3d.visualization.draw_geometries([source_temp, target_temp, frame_source, frame_origin])

def preprocess_source_pcd(source, voxel_size, scale_factor = 1.0/1000):
    
    # Scale and downsample is already done when we load the model
    print(":: Preprocess CAD point cloud")
    
    #source = scale_point_cloud(cloud=source, scale_factor=scale_value)

    #source_down = downsample_point_cloud(cloud=source, voxel_size=0.008)
    source_down = copy.deepcopy(source)

    radius_normal = voxel_size * 2
    print("   Estimate normals with search radius %.3f." % radius_normal)
    source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print("   Compute FPFH feature with search radius %.3f." % radius_feature)
    source_fpfh = o3d.registration.compute_fpfh_feature(source, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    source_down_fpfh = o3d.registration.compute_fpfh_feature(source_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    return source, source_down, source_fpfh, source_down_fpfh

def preprocess_point_cloud(cloud, voxel_size):

    print(":: Preprocess target point cloud")
    pcd = copy.deepcopy(cloud)

    print("   Downsample with a voxel size %.3f." % voxel_size)
    #pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down = copy.deepcopy(cloud)

    radius_normal = voxel_size * 2
    print("   Estimate normals with search radius %.3f." % radius_normal)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    
    radius_feature = voxel_size * 5
    print("   Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    pcd_down_fpfh = o3d.registration.compute_fpfh_feature(pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    return pcd, pcd_down, pcd_fpfh, pcd_down_fpfh

def execute_global_registration(source, target, source_fpfh, target_fpfh,
                                voxel_size):

    max_correspondence_distance = voxel_size * 0.5 # before was 1.5
    estimation_method = o3d.registration.TransformationEstimationPointToPoint(False)
    #estimation_method = o3d.registration.TransformationEstimationPointToPlane() PointToPlane is not working!!!
    ransac_n = 4
    edge_length = 0.9
    checkers = [o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(edge_length),
                o3d.registration.CorrespondenceCheckerBasedOnDistance(max_correspondence_distance)]
    #           CorrespondenceCheckerBasedOnNormal
    max_iteration = 4000000 # with 100000 didnt work well, with 4000000 takes too much time but works
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

def execute_local_registration(source, target, source_fpfh, target_fpfh, result_ransac, voxel_size):

    start_local = time.time()
    max_correspondence_distance = voxel_size * 0.4 # before was 0.4
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