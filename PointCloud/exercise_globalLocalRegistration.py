#!/usr/bin/env python
# coding: utf-8

# # Global registration with RANSAC
# We are going to use open3d (http://www.open3d.org/) to handle  pointclouds and generation of pointclouds
# 
# So make sure to call **pip install open3d**
# 

# In[1]:


import open3d as o3d
pcd = o3d.io.read_point_cloud("m200.ply")
o3d.io.write_point_cloud("m200.pcd", pcd)


# In[2]:


import open3d as o3d
import numpy as np
import copy

# helper function for drawing if you want it to be 
# more clear which is which set recolor=True
def draw_registrations(source, target, transformation = None, recolor = False):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    if(recolor):
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
    if(transformation is not None):
        source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


# We need to read in our pointclouds.

# In[3]:


#source = o3d.io.read_point_cloud("ICP/r1.pcd")
source = o3d.io.read_point_cloud("m200_pcd.pcd")
target = o3d.io.read_point_cloud("ICP/r3.pcd")

# Show models side by side
draw_registrations(source, target, None, True)


# ### Finding features in pointclouds
# When working on point clouds it can be benefitial work on a downsampled version of the point cloud.
# you can use [```pointcloudname.voxel_down_sample()```](http://www.open3d.org/docs/latest/python_api/open3d.geometry.PointCloud.html) where pointcloud is the name of your point cloud object.
# 
# We also need to estimate the normals of the pointcloud points using [```pointcloudname.estimate_normals()```](http://www.open3d.org/docs/latest/python_api/open3d.geometry.PointCloud.html)
# 
# And finally find fpfh features or correspondance of the downsampled point clouds.
# [```o3d.registration.compute_fpfh_feature()```](http://www.open3d.org/docs/latest/python_api/open3d.registration.compute_fpfh_feature.html#open3d.registration.compute_fpfh_feature)
# 
# 
# Global and Local registration tutorial: http://www.open3d.org/docs/release/tutorial/Advanced/global_registration.html

# ### Ransac
# We will now attempt to use ransac to do a global registration of the two poinclouds.
# 
# Using the function [```o3d.registration.registration_ransac_based_on_feature_matching```](http://www.open3d.org/docs/latest/python_api/open3d.registration.registration_ransac_based_on_feature_matching.html#open3d.registration.registration_ransac_based_on_feature_matching) from open3d
# 
# 
# Try to find the transformation from r1 to r2.
# ```Python
# point_to_point =  o3d.registration.TransformationEstimationPointToPoint(False)
# ```
# 
# When using ransac focus on the arguments below the rest are optional
# ```Python
# ransac_result = o3d.registration.registration_ransac_based_on_feature_matching(
#     source_sample, target_sample, 
#     source_fpfh, target_fpfh, 
#     distance_threshold,
#     point_to_point)
# ```

# ## Exercises
# ### A)
#     Can you get a decent transformation from r1 to r3?
# ### B)
#     Try to use pruning to stop Ransac early. A pruning step takes fast pruning algorithms to quickly reject false matches early.
# 
# Open3D provides the following pruning algorithms:
# 
# - **CorrespondenceCheckerBasedOnDistance** checks if aligned point clouds are close (less than specified threshold).
# 
# - **CorrespondenceCheckerBasedOnEdgeLength** checks if the lengths of any two arbitrary edges (line formed by two vertices) individually drawn from source and target correspondences are similar. This tutorial checks that ||edge_source||>0.9×||edge_target|| and ||edge_target||> 0.9×||edge_source|| are true.
# 
# - **CorrespondenceCheckerBasedOnNormal** considers vertex normal affinity of any correspondences. It computes dot product of two normal vectors. It takes radian value for the threshold.
# 
# 
# You can also try tweaking the voxel_size
# ```Python
# corr_length = 0.9
# distance_threshold = voxel_size * 1.5
# 
# # Checkers
# c0 = o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(corr_length)
# c1 = o3d.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
# c2 = o3d.registration.CorrespondenceCheckerBasedOnNormal(0.095)
# 
# checker_list = [c0, c1, c2]
# 
# ransac_result = o3d.registration.registration_ransac_based_on_feature_matching(
#     source_sample, target_sample, 
#     source_fpfh, target_fpfh, 
#     distance_threshold,
#     point_to_point,
#     checkers = checker_list)
# ```
# 
# ### D)
# Try to use **RANSACConvergenceCriteria** to see how many iterations are needed for decent convergence for both point to point and point to plane.
# 
# Replace point_to_point with point_to_plane.
# ```Python
# point_to_plane =  o3d.registration.TransformationEstimationPointToPlane()
# 
# crit = o3d.registration.RANSACConvergenceCriteria(1000000, 100)
# 
# ransac_result = o3d.registration.registration_ransac_based_on_feature_matching(
#     source_sample, target_sample, 
#     source_fpfh, target_fpfh, 
#     distance_threshold,
#     point_to_plane,
#     checkers = checker_list
#     criteria = crit)
# ```

# In[6]:


#### GLOBAL REGISTRATION

# For exercise A: match r1.pcd and r3.pcd
#print("Reading source and target point clouds")
#source = o3d.io.read_point_cloud("ICP/r1.pcd")
#target = o3d.io.read_point_cloud("ICP/r3.pcd")

print("Reading source and target point clouds")
source = o3d.io.read_point_cloud("ICP/r1.pcd")
target = o3d.io.read_point_cloud("ICP/r3.pcd")

voxel_size = 0.05
print("Downsampling the point clouds")
source_down = source.voxel_down_sample(voxel_size)
target_down = target.voxel_down_sample(voxel_size)

radius_normal = voxel_size * 2
print("Estimate normal with search radius %.3f." % radius_normal)
source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

radius_feature = voxel_size * 5
print("Compute FPFH feature with search radius %.3f." % radius_feature)
source_fpfh = o3d.registration.compute_fpfh_feature(source_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
target_fpfh = o3d.registration.compute_fpfh_feature(target_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

corr_length = 0.9
distance_threshold = voxel_size * 1.5

# Checkers
c0 = o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(corr_length)
c1 = o3d.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
c2 = o3d.registration.CorrespondenceCheckerBasedOnNormal(0.095)
checkers = [c0, c1, c2]

point_to_point = o3d.registration.TransformationEstimationPointToPoint(False)
#point_to_plane = o3d.registration.TransformationEstimationPointToPlane()

criteria = o3d.registration.RANSACConvergenceCriteria(1000000, 100)
#criteria = o3d.registration.RANSACConvergenceCriteria(4000000, 500)

print("RANSAC algorithm")
result_ransac = o3d.registration.registration_ransac_based_on_feature_matching(source_down, target_down,
                                                                               source_fpfh, target_fpfh,
                                                                               distance_threshold, point_to_point,
                                                                               4,checkers, criteria)

draw_registrations(source, target, result_ransac.transformation, True)


# In[5]:


#### LOCAL REGISTRATION

#print("Point-to-point ICP registration")
#p2p = o3d.registration.TransformationEstimationPointToPoint(False)
#result_icp = o3d.registration.registration_icp(source, target, distance_threshold,
#                                              result_ransac.transformation, p2p)

# If we want to use point_to_plane we have to get the normals as an input (source_down and target_down)
print("Point-to-plane ICP registration")
p2plane = o3d.registration.TransformationEstimationPointToPlane()
result_icp = o3d.registration.registration_icp(source_down, target_down, distance_threshold, result_ransac.transformation, p2plane)

draw_registrations(source, target, result_icp.transformation, False)
print("DONE")

