#! /usr/bin/env python

import rospy
import numpy as np
import open3d as o3d
from open3d import *
import matplotlib.pyplot as plt
from sklearn import cluster
import copy
import sys
import eigenpy

from functions import *


if __name__ == "__main__":

    rospy.init_node('listener', anonymous=True)
    
    filename = sys.argv[1]
    pcd_input = o3d.io.read_point_cloud(filename)
    print(":: Read Point Cloud with " + str(len(pcd_input.points)) + " points" )

    pcd = copy.deepcopy(pcd_input)
    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    data_frame = o3d.geometry.TriangleMesh()

    display_flag = True
    bounding_boxes = []
    
    # Threshold data with Z filter
    if (sys.argv[2] == "threshold"):
        pcd = threshold_filter_z(cloud=pcd, z_distance=0.49, display=display_flag)

    # Segment the plane with RANSAC
    if (sys.argv[3] == "segment_plane"):
        plane_cloud, pcd = segment_plane_ransac(cloud=pcd, distance_threshold=0.01, ransac_n=3, num_iterations=100, display=display_flag)

    # DBSCAN clustering 
    if (sys.argv[4] == "cluster"):
        pcd_clustered, nb_clusters, pcd = cluster_dbscan(cloud=pcd, eps=0.01, min_samples=400, display=False)
        
        for i in range(nb_clusters):

            print(">> Cluster %d: %d points" %(i, len(pcd_clustered[i].points)))

            box = pcd_clustered[i].get_oriented_bounding_box()
            box.color = (1,0,0)
            bounding_boxes.append(box)

            box_points = box.get_box_points()
            box_points = np.asarray(box_points)
            #print(box_points)

            xyz = pcd.points
            xyz_3dvector = o3d.utility.Vector3dVector(xyz)

            #pcd_temp = pcd.crop(bounding_box=box)
            #print(pcd_temp.is_empty())
            ind_cluster = box.get_point_indices_within_bounding_box(xyz_3dvector)

            #print(len(ind_cluster))
            pcd_clusters_left = pcd.select_down_sample(indices=ind_cluster, invert=True)
            pcd_clusters_left.paint_uniform_color([1,0,0])

            #o3d.visualization.draw_geometries([pcd_clusters_left, box])

        #vol = o3d.visualization.read_selection_polygon_volume("bounding_box.json")
        #pcd_temp = vol.crop_point_cloud(pcd)
        #o3d.visualization.draw_geometries([pcd_temp])

        pcd_clusters_boxes = pcd_clustered + bounding_boxes
        o3d.visualization.draw_geometries(pcd_clusters_boxes)

    ## Options for CAD models
    # scale data
    if (sys.argv[5] == "scale"):
        pcd = scale_point_cloud(cloud=pcd, scale_factor=1.0/1000)

    # downsample data
    if (sys.argv[6] == "downsample"):
        pcd = downsample_point_cloud(cloud=pcd, voxel_size=0.008)

    if (sys.argv[7] == "save"):
        o3d.io.write_point_cloud("saved_" + sys.argv[5] + "_" + sys.argv[6] + "_" + filename, pcd)

    # show origin frame of the data
    if (sys.argv[8] == "frame_cad"):
        data_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=pcd.get_center())

    pcd_output = copy.deepcopy(pcd)
    o3d.visualization.draw_geometries([pcd_output, origin_frame, data_frame]) # press n to see point normal

    #rospy.spin()
    print("DONE")
