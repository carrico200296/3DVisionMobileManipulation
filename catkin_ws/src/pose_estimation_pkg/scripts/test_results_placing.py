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
import cv_bridge
import geometry_msgs.msg
import sensor_msgs
import std_msgs
from scipy.spatial.transform import Rotation as R

from functions import *

import rtde_control
import rtde_receive
import xlsxwriter


def insert_component(rtde_c, rtde_r, placing_pose, fix_center):
    # Move to placing_pose
    rtde_c.moveL(placing_pose, 0.05, 0.05)
    # Insert component inside the fixture
    insert_pose = rtde_r.getActualTCPPose()
    insert_pose[2] = insert_pose[2] - 0.05
    rtde_c.moveL(insert_pose, 0.01, 0.01)
    time.sleep(2.0)
    # Move up
    rtde_c.moveL(placing_pose, 0.1, 0.1)
    end_point = fix_center
    end_point[2] = 0.5
    rtde_c.moveL(end_point, 0.1, 0.1)
    take_image_pose_joints = [0.4279041290283203, -1.8154250584044398, -1.332542896270752, -1.5645697873881836, 1.572350025177002, 0.7428669929504395]
    rtde_c.moveJ(take_image_pose_joints, 0.8, 0.8)
    time.sleep(1.0)

def detect_fixture_pose(fixture_model, camera_type, static_broadcaster, tfBuffer):
    bridge = cv_bridge.CvBridge()
    data_color = rospy.wait_for_message("/camera/color/image_raw", sensor_msgs.msg.Image, rospy.Duration(4.0))
    color_image = bridge.imgmsg_to_cv2(data_color, desired_encoding="passthrough")
    img_markers, markerCorners, markerIds = detect_aruco_markers(img=color_image, display=False)
    print(markerIds)
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

    file_time = str(int(time.time()))
    file_name = '/home/carlos/Desktop/samples_for_report/results/test_placing_aruco_calibration/sample_data_' + file_time + '.xlsx' 
    workbook = xlsxwriter.Workbook(file_name)
    worksheet = workbook.add_worksheet()
    row = 0
    column = 0
    headers = ["Sample", "Aruco_X", "Aruco_Y", "Aruco_Z", "Center_fixture_X", "Center_fixture_Y", "Center_fixture_Z", "Placed correctly"]
    for item in headers:
        worksheet.write(row, column, item)
        column+=1

    rospy.init_node('fixture_detection_node')
    base_frame = "base"

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    rtde_c = rtde_control.RTDEControlInterface("192.168.12.245")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.12.245")

    x_fixture_offset = 0.0415526468475
    y_fixture_offset = -0.0405504113613
    height_fixture = 0.150

    x_placing_offset = -0.11726419762
    y_placing_offset = -0.0231597946661
    z_placing_offset = 0.245 # it is a good distance

    tool_offset = 0.1245
    z_picking_offset = 0.130 #0.1315 before, all the calculations have been done with 0.130
    z_offset_center_fixture = tool_offset + height_fixture + 0.01

    # Localize Fixture - take_image_pose: predefined_pose for taking the image, accuracy based on MIR docking position
    #take_image_pose_joints = [0.4279041290283203, -1.8154250584044398, -1.332542896270752, -1.5645697873881836, 1.572350025177002, 0.7428669929504395]
    #rtde_c.moveJ(take_image_pose_joints, 0.8, 0.8)
    time.sleep(3.0)
    detect_fixture_pose(fixture_model="200", camera_type="D415", static_broadcaster=static_broadcaster, tfBuffer=tfBuffer)
    tf_base_fixture200 = tfBuffer.lookup_transform(base_frame, "depth_aruco_id_200", rospy.Time(), rospy.Duration(2.0))
    t, rot_vector = get_t_rotvector_target(tf_transform_base_target=tf_base_fixture200)
    time.sleep(1.0)
    print("Aruco marker position: ")
    print(t)

    sample_data = []
    sample_data.append(file_time)
    sample_data.append(t[0])
    sample_data.append(t[1])
    sample_data.append(t[2])

    fix_center = [t[0] + x_fixture_offset, t[1] + y_fixture_offset, z_offset_center_fixture, 2.908, -1.188, 0.0]
    print("Fixture center: ")
    print(fix_center)
    sample_data.append(fix_center[0])
    sample_data.append(fix_center[1])
    sample_data.append(fix_center[2])

    placing_pose = [fix_center[0] + x_placing_offset, fix_center[1] + y_placing_offset, z_placing_offset, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    print("Placing pose calculated: ")
    print(placing_pose)

    row+=1
    column = 0
    for item in sample_data:
        worksheet.write(row, column, item)
        column+=1

    workbook.close()

    # Place component
    insert_component(rtde_c=rtde_c, rtde_r=rtde_r, placing_pose = placing_pose, fix_center = fix_center)

    rtde_c.stopScript()
    rtde_c.disconnect()

    quit()
    rospy.spin()