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
import std_msgs
from scipy.spatial.transform import Rotation as R

from functions import *

import rtde_control
import rtde_receive

def pick_component(rtde_c, rtde_r, t, rot_vector, tool_offset, z_picking_offset):
    off_set = tool_offset + 0.08
    # Move to the picking-entry pose
    picking_entry_pose = [t[0], t[1], t[2] + off_set, rot_vector[0], rot_vector[1], rot_vector[2]]
    rtde_c.moveL(picking_entry_pose, 0.2, 0.2)
    # Open gripper
    rtde_c.gripperDisable()
    rtde_c.gripperRelease(0)
    # Move to picking_pose (z = 0.130 for m200 component)
    picking_pose = rtde_r.getActualTCPPose()
    picking_pose[2] = z_picking_offset
    rtde_c.moveL(picking_pose, 0.05, 0.1)
    time.sleep(0.2)
    # Close gripper
    rtde_c.gripperGrasp(0)
    time.sleep(1)
    # Move to picking-entry pose
    rtde_c.moveL(picking_entry_pose, 0.2, 0.1)

def place_component(rtde_c, rtde_r, placing_pose):
    # Move to placing_pose
    rtde_c.moveL(placing_pose, 0.2, 0.3)
    time.sleep(0.3)
    # Open gripper
    rtde_c.gripperRelease(0)
    time.sleep(1.0)

def detect_fixture_pose(fixture_model, camera_type, static_broadcaster, tfBuffer):
    data_color = rospy.wait_for_message("/camera/color/image_raw", sensor_msgs.msg.Image)
    color_image = bridge.imgmsg_to_cv2(data_color, desired_encoding="passthrough")
    img_markers, markerCorners, markerIds = detect_aruco_markers(img=color_image, display=False)

    if markerIds is not None:
        img_axis, rvecs, tvecs = detect_marker_pose(img=img_markers, corners=markerCorners, ids=markerIds, camera_type=camera_type, display=False)
        
        for i in range(len(markerIds)):
            if str(markerIds[i][0]) == fixture_model:
                color_aruco_frame = "color_aruco_id_" + str(markerIds[i][0])
                broadcaster_aruco_color_frame(static_broadcaster, rvecs[i], tvecs[i], color_aruco_frame)
                transformStamped = tfBuffer.lookup_transform("camera_depth_optical_frame", color_aruco_frame, rospy.Time(), rospy.Duration(0.5))
                depth_aruco_frame = "depth_aruco_id_" + str(markerIds[i][0])
                broadcaster_aruco_depth_frame(static_broadcaster, transformStamped, depth_aruco_frame)


if __name__ == "__main__":
 
    rospy.init_node('manipulation_node')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    nb_components = rospy.wait_for_message("/pose_estimation/nb_components", std_msgs.msg.String, rospy.Duration(900.0))
    rtde_c = rtde_control.RTDEControlInterface("192.168.12.245")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.12.245")

    base_frame = "base"
    print(":: Number detected components = %d" %(int(nb_components.data)))

    list_transform_base_component = []
    for i in range(int(nb_components.data)):
        component_frame = "object_frame_" + str(i+1)
        list_transform_base_component.append(tfBuffer.lookup_transform(base_frame, component_frame, rospy.Time(0)))

    for i in range(int(nb_components.data)):
        tf_transform_base_component = list_transform_base_component[i]
        t, rot_vector = get_t_rotvector_component(tf_transform_base_component=tf_transform_base_component)

        tool_offset = 0.1245
        z_picking_offset = 0.130 #0.1315 before, all the calculation are than with 0.130

        # Pick Component
        pick_component(rtde_c, rtde_r, t, rot_vector, tool_offset, z_picking_offset)

        # Localize Fixture
        
        # Place component
        #fixture_pose = [0.1974, 0.18591, 0.24911, 2.908, -1.188, 0.0]
        place_component(rtde_c=rtde_c, rtde_r=rtde_r, placing_pose = fixture_pose)

    rtde_c.gripperDisable()
    rtde_c.stopScript()
    rtde_c.disconnect()

    quit()
    rospy.spin()