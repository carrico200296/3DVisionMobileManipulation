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

'''
num_of_pcds = int(sys.argv[1])
pcd = o3d.geometry.PointCloud()
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

for i in range(num_of_pcds):
    filename = str(i) + ".pcd"
    pcd_temp = o3d.io.read_point_cloud(filename)
    frame = frame + o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=pcd_temp.get_center())
    #pcd_temp = pcd_temp.translate((0,0,0))
    pcd = pcd + pcd_temp

# threshold data
points = np.asarray(pcd.points)
pcd_sel = pcd.select_by_index(np.where(points[:, 2] < 0.8)[0])

# visualize different point clouds
o3d.visualization.draw_geometries([pcd_sel, frame])

'''

if __name__ == "__main__":
 
    scene_pcd = o3d.geometry.PointCloud()
    rospy.init_node('scene_reconstruction')

    data = rospy.wait_for_message("/camera/depth/color/points", sensor_msgs.msg.PointCloud2)
    scene_pcd = orh.rospc_to_o3dpc(data, remove_nans=True)
    print("   Scene Point Cloud has " + str(len(scene_pcd.points)) + " points")

    ## THRESHOLDING PARAMETERS
    # default values (maybe it will change depending on the view)
    ## IDEA: make the region of interest depending on the ARUCO codes
    #roi = rospy.wait_for_message("/aruco/detected_roi", np.array()) we can get the ROI in the 3D space from the topic /aruco/detected_roi
    z_distance = 0.49
    distance_threshold = 0.007

    scene_pcd = threshold_filter_z(cloud=scene_pcd, z_distance=z_distance, display=False)
    scene_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    o3d.visualization.draw_geometries([scene_pcd])

    rospy.spin()
    print("DONE")
