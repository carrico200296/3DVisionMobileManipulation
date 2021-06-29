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

def pick_component(rtde_c, rtde_r, t, rot_vector, tool_offset, z_picking_offset):
    off_set = tool_offset + 0.08
    # Move to the picking-entry pose
    picking_entry_pose = [t[0], t[1], t[2] + off_set, rot_vector[0], rot_vector[1], rot_vector[2]]
    rtde_c.moveL(picking_entry_pose, 0.2, 0.2)
    # Open gripper
    rtde_c.gripperDisable()
    rtde_c.gripperRelease(0)
    # Move to picking_pose (z = 0.130 for Housing component)
    picking_pose = rtde_r.getActualTCPPose()
    picking_pose[2] = z_picking_offset
    print("Component picking Point:")
    print(picking_pose)
    rtde_c.moveL(picking_pose, 0.1, 0.1)
    # Close gripper
    rtde_c.gripperGrasp(0)
    time.sleep(1.5)
    # Move to picking-entry pose
    rtde_c.moveL(picking_entry_pose, 0.2, 0.2)

def place_component(rtde_c, rtde_r, placing_pose, fix_center):
    # Move to placing_pose
    rtde_c.moveL(placing_pose, 0.4, 0.4)

    start_poseQ = rtde_r.getActualQ()
    start_pose = rtde_r.getActualTCPPose()
    end_pose = start_pose[:]
    end_pose[2]= end_pose[2] - 0.08
    flag = False
    required_rotation = False
    rtde_c.moveL(end_pose, 0.01, 0.01, True) 
    while not flag:
        direction = rtde_r.getTargetTCPSpeed()
        flag_contact = rtde_c.toolContact(direction)
        current_pose = rtde_r.getActualTCPPose()
        distance = start_pose[2] - current_pose[2]
        if flag_contact > 0:
            rtde_c.stopL(1.5)
            flag = True
            required_rotation = True
        if distance > 0.07:
            flag = True

    flag = False
    if required_rotation == True:
        rtde_c.moveL(start_pose, 0.1, 0.1)
        rotated_pose = start_poseQ[:]
        rotated_pose[5] = -3.1416121164904993 # to rotate 180 degrees the wrist
        rtde_c.moveJ(rotated_pose, 0.8, 0.8)
        # Insert component inside the fixture
        insert_pose = rtde_r.getActualTCPPose()
        insert_pose[2] = insert_pose[2] - 0.08
        rtde_c.moveL(insert_pose, 0.01, 0.01, True)
        while not flag:
            direction = rtde_r.getTargetTCPSpeed()
            flag_contact = rtde_c.toolContact(direction)
            current_pose = rtde_r.getActualTCPPose()
            distance = start_pose[2] - current_pose[2]
            if flag_contact > 0:
                rtde_c.stopL(1.5)
                flag = True
            if distance > 0.07:
                flag = True
    
    # Open gripper
    rtde_c.gripperDisable()
    rtde_c.gripperRelease(0)
    time.sleep(1.5)

    pose = rtde_r.getActualTCPPose()
    pose[2] = pose[2] + 0.2
    rtde_c.moveL(pose, 0.2, 0.2)
    rtde_c.moveJ(take_image_pose_joints, 0.8, 0.8)

def detect_aruco_pose(fixture_model, camera_type, static_broadcaster, tfBuffer):
    bridge = cv_bridge.CvBridge()
    data_color = rospy.wait_for_message("/camera/color/image_raw", sensor_msgs.msg.Image, rospy.Duration(4.0))
    color_image = bridge.imgmsg_to_cv2(data_color, desired_encoding="passthrough")
    img_markers, markerCorners, markerIds = detect_aruco_markers(img=color_image, display=False)
    print(markerIds)
    if markerIds is not None:
        img_axis, rvecs, tvecs = detect_marker_pose(img=img_markers, corners=markerCorners, ids=markerIds, camera_type=camera_type, display=False)
        cv2.imwrite("/home/carlos/Desktop/samples_for_report/results/video_defense/aruco_fixture_detection.png", img_axis)

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

    nb_components = rospy.wait_for_message("/pose_estimation/nb_components", std_msgs.msg.String, rospy.Duration(1200.0))
    pub_feeding_task_status = rospy.Publisher("/pose_estimation/status", std_msgs.msg.String, queue_size=1)

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
        t, rot_vector, y_axis_original = get_t_rotvector_component(tf_transform_base_component=tf_transform_base_component)

        print(":: Pose Component %d" %(i+1))
        print("   Translation:")
        print(t)
        print("   Rotation Axis:")
        print(rot_vector)

        # Offset constant parameters
        tool_offset = 0.1245
        component_offset = 0.005

        z_picking_offset = 0.130
        height_fixture = 0.150
        z_offset_center_fixture = tool_offset + height_fixture + 0.01
        z_placing_offset =  0.245

        # Pick Component
        pick_component(rtde_c, rtde_r, t, rot_vector, tool_offset, z_picking_offset)

        # Localize Fixture - take_image_pose: predefined_pose for taking the image, accuracy based on MIR docking position
        take_image_pose_joints = [0.4279041290283203, -1.8154250584044398, -1.332542896270752, -1.5645697873881836, 1.572350025177002, 0.7428669929504395]
        rtde_c.moveJ(take_image_pose_joints, 0.8, 0.8)

        #if i == 0:
        time.sleep(3.0)
        detect_aruco_pose(fixture_model="200", camera_type="D415", static_broadcaster=static_broadcaster, tfBuffer=tfBuffer)
        tf_base_marker = tfBuffer.lookup_transform(base_frame, "depth_aruco_id_200", rospy.Time(), rospy.Duration(2.0))
        t, rot_vector, base_T_marker = get_t_rotvector(tf_transform_base_target=tf_base_marker)
        time.sleep(1.0)
        print("Aruco marker position: ")
        print(t)
        print("Aruco marker orientation: ")
        print(rot_vector)

        # Calculate the center of the fixture
        marker_T_fixture = np.eye(4)
        marker_T_fixture[1,3] = 0.058 
        marker_T_fixture[0,3] = 0.003
        base_T_fixture = np.linalg.multi_dot([base_T_marker, marker_T_fixture])
        fix_center = [base_T_fixture[0,3], base_T_fixture[1,3], z_offset_center_fixture, 2.908, -1.188, 0.0]
        print("Fixture center: ")
        print(fix_center)

        # Calulate the placing pose
        fixture_T_placing = np.eye(4)
        fixture_T_placing[1,3] = -tool_offset + component_offset
        base_T_placing = np.linalg.multi_dot([base_T_marker, marker_T_fixture, fixture_T_placing])
        placing_pose = [base_T_placing[0,3], base_T_placing[1,3], z_placing_offset, -0.762054625026805, 1.7619667569065234, -1.774435905185706]

        print("Placing pose calculated: ")
        print(placing_pose)

        # Place component
        place_component(rtde_c=rtde_c, rtde_r=rtde_r, placing_pose = placing_pose, fix_center = fix_center)

    rtde_c.gripperDisable()

    home_pose = [0.24629746314958142, 0.39752299707520433, 0.5357813117643294, 2.890757991539826, -1.230076763649183, 1.296199316724078e-05]
    rtde_c.moveL(home_pose, 0.4, 0.5)
    rtde_c.stopScript()
    rtde_c.disconnect()
    pub_feeding_task_status.publish("feeding_task_done")

    quit()