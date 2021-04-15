import open3d as o3d
import numpy as np
import copy
import sys
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import sklearn

from functions import *

filename = sys.argv[1]
pcd = o3d.io.read_point_cloud(filename)
print(pcd)
distance_threshold = 0.000009

#_, pcd = segment_plane_ransac(cloud=pcd, distance_threshold=0.00009, ransac_n=3, num_iterations=100, display=False)
pcd = threshold_filter_min_max(pcd, axis=0, min_distance=-0.15, max_distance=0.15)
pcd = threshold_filter_min_max(pcd, axis=1, min_distance=-0.15, max_distance=0.15)
pcd = threshold_filter_min_max(pcd, axis=2, min_distance=0.3, max_distance=0.53)
#plane, pcd = segment_plane_ransac(cloud=pcd, distance_threshold=distance_threshold, ransac_n=3, num_iterations=100, display=False)
pcd, inliers = remove_color_outlier(cloud=pcd, color_threshold=200)

frame_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
o3d.visualization.draw_geometries([pcd])

# DBSCAN Clustering
#scene_pcd_clustered, nb_clusters, scene_pcd = cluster_dbscan(cloud=pcd, eps=0.01, min_samples=400, min_points_cluster = 3000, display=False)
scene_pcd_clustered, nb_clusters, scene_pcd = cluster_dbscan(cloud=pcd, eps=0.01, min_samples=100, min_points_cluster = 1000, display=False)

print(":: Final Scene Clusters to register:")
for cluster in range(nb_clusters):
    nb_cluster_points = len(scene_pcd_clustered[cluster].points)
    print("   Cluster %d: %d points" %(cluster + 1, nb_cluster_points))

o3d.visualization.draw_geometries(scene_pcd_clustered)




#print("Testing kdtree in Open3D...")
#pcd.paint_uniform_color([0.5, 0.5, 0.5])
'''
pcd_tree = o3d.geometry.KDTreeFlann(pcd)
pcd.colors[11500] = [1, 0, 0]
print("Find its 3 nearest neighbors, and paint them blue.")
[k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[11500], 3)
np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]
'''

#pcd = filter_pcd(pcd)
#o3d.visualization.draw_geometries([pcd])

'''
#pcd = filter_pcd(pcd, x_distance=0.05, y_distance=0.03)
#print(pcd)
#o3d.visualization.draw_geometries([pcd])
distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
print(avg_dist)
print(pcd.has_colors())
print(pcd.has_normals())
print(pcd.has_points())

# remove_color_outlier: Funtion to remove all the points with R,G and B color value < than a color_threshold
#pcd_thresholded, inliers = remove_color_outlier(cloud=pcd, color_threshold=170)
#o3d.visualization.draw_geometries([pcd_thresholded])
#display_inlier_outlier(pcd, inliers)

# remove_statistical_outlier: Funtion to remove points that are further away from their neighbors compared to the average for the point cloud
#pcd_thresholded, ind = pcd.remove_statistical_outlier(nb_neighbors = 20, std_ratio = 3.0) # std_ratio: The lower this number the more aggressive the filter will be.
#display_inlier_outlier(pcd_thresholded, ind)

# remove_radius_outlier: Function to remove points that have less than nb_points in a given sphere of a given radius
#pcd_thresholded, ind = pcd.remove_radius_outlier(nb_points=1, radius=0.00001)
'''