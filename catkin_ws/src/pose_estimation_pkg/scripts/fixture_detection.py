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
import multiprocessing



def place_component(rtde_c, rtde_r, placing_pose, fix_center):
    # Move to placing_pose
    rtde_c.moveL(placing_pose, 0.05, 0.05)
    # Insert component inside the fixture
    insert_pose = rtde_r.getActualTCPPose()
    insert_pose[2] = insert_pose[2] - 0.05
    rtde_c.moveL(insert_pose, 0.01, 0.01)
    # Open gripper
    rtde_c.gripperRelease(0)
    time.sleep(2.0)
    # Move to pick another component
    rtde_c.moveL(placing_pose, 0.1, 0.1)
    end_point = fix_center
    end_point[2] = 0.405
    rtde_c.moveL(end_point, 0.1, 0.1)
    rtde_c.gripperDisable()
    rtde_c.gripperGrasp(0)
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
 
    rospy.init_node('fixture_detection_node')
    base_frame = "base"

    '''
    print(":: Loading cad_model point cloud.")
    cad_file_path = "/home/carlos/git/3DVisionMobileManipulation/catkin_ws/src/pose_estimation_pkg/data/scaled_downsampled_m200.pcd"
    cad_model_pcd = o3d.io.read_point_cloud(cad_file_path, print_progress=True)
    cad_model_pcd.translate(translation=(0, 0, 0), relative=False)
    origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    R = origin_frame.get_rotation_matrix_from_xyz((np.pi / 2, 0, 0))
    cad_model_pcd = threshold_filter_circle(cloud=cad_model_pcd, radius=0.009, display=False)
    cad_model_pcd.rotate(R, center=False)
    rotated_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.06)
    rotated_frame.rotate(R, center=False)
    o3d.visualization.draw_geometries([cad_model_pcd, origin_frame])#, rotated_frame])
    '''

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    rtde_c = rtde_control.RTDEControlInterface("192.168.12.245")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.12.245")

    #ARUCO-FIXTURE OFFSET CALCULATION (to calculate statistics)
    # "sample_x" for Aruco location detected "fixture_location_calculated" for ficture center detected
    #sample_1a = [0.7006637307671114, 0.1018380679219168, 0.2845, 2.908, -1.188, 0.0]
    #sample_2a = [0.7007150217732067, 0.1024004188919436, 0.2845, 2.908, -1.188, 0.0]
    #sample_3a = [0.7009458456988304, 0.10214223936068872, 0.0005296980402354767]
    #sample_4a = [0.7010068766864602, 0.1026414759430313, 0.003968304070235351]
    #sample_5b = [0.7050102704328021, 0.10198571376564713, -0.0008147989116149834]
    #sample_6b = [0.7051735731001214, 0.10217736747358186, -0.000749306057693383]
    #sample_7b = [0.7052084922717082, 0.10240982070211074, 0.0002507968076939926]
    #sample_8b = [0.705264254931799, 0.10246322844920475, -0.00032096330264136386]
    #sample_9b = [0.7053362604185026, 0.10255897003907234, 0.0007246964764954511]
    #sample_10b = [0.7053364273857389, 0.10254730766242935, 0.0010560746718380476]
    #fixture_location_calculated_sample_6b = [0.7467262199476215, 0.06162695611228186, 0.2845, 2.908, -1.188, 0.0]
    #fixture_location_calculated_sample_7b = [0.7467611391192083, 0.061859409340810745, 0.2845, 2.908, -1.188, 0.0]
    #fixture_location_calculated_sample_8b = [0.746816901779299, 0.06191281708790475, 0.2845, 2.908, -1.188, 0.0]
    #fixture_location_calculated_sample_9b = [0.7468889072660027, 0.062008558677772344, 0.2845, 2.908, -1.188, 0.0]
    #fixture_location_calculated_sample_10b = [0.7468890742332389, 0.06199689630112935, 0.2845, 2.908, -1.188, 0.0]

    # fixture_location_manual_a = [0.7422676686207003, 0.06185000753060148, 0.315, 2.9080196434959547, -1.1879900911879482, 2.2863347736277715e-05]
    # fixture_location_manual_b = [0.7459383594566255, 0.061439219999072335, 0.315, 2.9077471717816645, -1.188803675694078, 3.450345541009959e-05]

    #>>> diff_x = fixture_location_manual[0] - sample_2[0]
    #>>> print(diff_x)
    #0.0415526468475
    #>>> diff_y = fixture_location_manual[1] - sample_2[1]
    #>>> print(diff_y)
    #-0.0405504113613
    x_fixture_offset = 0.0415526468475
    y_fixture_offset = -0.0405504113613
    height_fixture = 0.150

    #FIXTURE_CENTER - PLACING ORIENTED POSE OFFSET CALCULATION
    #center_fixture_oriented = [0.742260932217375, 0.06183793596849685, 0.22860847605029044, -1.2878227766872377, 1.0705819504509524, -1.074506410798968]
    #center_fixture_offset_oriented = [0.6249967345976565, 0.03867814130236337, 0.2286438278375378, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    
    #>>> diff_x = center_fixture_offset_oriented[0] - center_fixture_oriented[0] 
    #>>> print(diff_x)
    #-0.11726419762
    #>>> diff_y = center_fixture_offset_oriented[1] - center_fixture_oriented[1]
    #>>> print(diff_y)
    #-0.0231597946661

    # PLACING_POSE CALCULATED with VISION vs MANUAL (to calculate statistics) - related with the aruco marker detection samples
    # placing_pose_calculated_sample_3a = [0.6252342949263305, 0.03843203333328873, 0.228, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    # placing_pose_calculated_sample_4a = [0.6252953259139603, 0.038931269915631304, 0.228, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    # placing_pose_calculated_sample_5b = [0.6292987196603022, 0.03827550773824714, 0.228, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    # placing_pose_calculated_sample_6b = [0.6294620223276215, 0.03846716144618186, 0.228, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    # placing_pose_calculated_sample_7b = [0.6294969414992083, 0.038699614674710746, 0.228, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    # placing_pose_calculated_sample_8b = [0.6295527041592991, 0.03875302242180475, 0.228, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    # placing_pose_calculated_sample_9b = [0.6296247096460027, 0.038848764011672345, 0.228, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    # placing_pose_calculated_sample_10b = [0.629624876613239, 0.038837101635029354, 0.228, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    # placing_pose_manual_a = [0.6249967345976565, 0.03867814130236337, 0.2286438278375378, -1.287808247339734, 1.070439178974699, -1.074470886939796]

    x_placing_offset = -0.11726419762
    y_placing_offset = -0.0231597946661
    z_placing_offset = 0.245 # it is a good distance

    tool_offset = 0.1245
    z_picking_offset = 0.130 #0.1315 before, all the calculations have been done with 0.130
    z_offset_center_fixture = tool_offset + height_fixture + 0.01

    # Pick a component
    #entry_point_assembly_line = [0.5646680489343061, -0.14747084956424256, 0.1923333064137298, 1.2084273498616414, -2.8987764754231495, 0.0002400822177717899]
    entry_point = [0.40166859802430244, 0.31179738856044764, 0.14938715474534492, 2.9071470273265487, -1.186809924863102, 0.00028546423748938507]
    rtde_c.moveL(entry_point, 0.1, 0.1)
    rtde_c.gripperDisable()
    rtde_c.gripperRelease(0)
    #picking_point_assembly_line  = [0.564682488566143, -0.14747767132023368, 0.16774064035158953, 1.208439480512014, -2.8988074757401505, 0.00017254328108733615]
    picking_point = rtde_r.getActualTCPPose()
    picking_point[2] = z_picking_offset
    rtde_c.moveL(picking_point, 0.1, 0.1)
    rtde_c.gripperGrasp(0)
    time.sleep(1.5)
    rtde_c.moveL(entry_point, 0.2, 0.2)

    # Localize Fixture - take_image_pose: predefined_pose for taking the image, accuracy based on MIR docking position
    #take_image_pose = [0.5967660895688741, 0.1253897292211242, 0.47289248807659473, -1.845352220142729, 2.542406034245773, 1.789893252155485e-05]
    #rtde_c.moveL(take_image_pose, 0.5, 0.5)
    take_image_pose_joints = [0.4279041290283203, -1.8154250584044398, -1.332542896270752, -1.5645697873881836, 1.572350025177002, 0.7428669929504395]
    rtde_c.moveJ(take_image_pose_joints, 0.8, 0.8)
    time.sleep(3.0)
    detect_fixture_pose(fixture_model="200", camera_type="D415", static_broadcaster=static_broadcaster, tfBuffer=tfBuffer)
    tf_base_fixture200 = tfBuffer.lookup_transform(base_frame, "depth_aruco_id_200", rospy.Time(), rospy.Duration(2.0))
    t, rot_vector = get_t_rotvector_target(tf_transform_base_target=tf_base_fixture200)
    time.sleep(1.0)
    print("Aruco marker position: ")
    print(t)
    fix_center = [t[0] + x_fixture_offset, t[1] + y_fixture_offset, z_offset_center_fixture, 2.908, -1.188, 0.0]
    print("Fixture center: ")
    print(fix_center)
    placing_pose = [fix_center[0] + x_placing_offset, fix_center[1] + y_placing_offset, z_placing_offset, -1.287808247339734, 1.070439178974699, -1.074470886939796]
    print("Placing pose calculated: ")
    print(placing_pose)
    print("Placing pose manual (measure it in the teach pendant): ")

    # Move to placing_pose
    rtde_c.moveL(placing_pose, 0.1, 0.1)

    start_poseQ = rtde_r.getActualQ()
    start_pose = rtde_r.getActualTCPPose()
    end_pose = start_pose[:]
    end_pose[2]= end_pose[2] - 0.07
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
        if distance > 0.065:
            flag = True
    print(required_rotation)

    flag = False
    if required_rotation == True:
        rtde_c.moveL(start_pose, 0.1, 0.1)
        rotated_pose = start_poseQ[:]
        rotated_pose[5] = -3.1416121164904993 # to rotate 180 degrees the wrist
        rtde_c.moveJ(rotated_pose, 0.8, 0.8)
        # Insert component inside the fixture
        insert_pose = rtde_r.getActualTCPPose()
        insert_pose[2] = insert_pose[2] - 0.07
        rtde_c.moveL(insert_pose, 0.01, 0.01, True)
        while not flag:
            direction = rtde_r.getTargetTCPSpeed()
            flag_contact = rtde_c.toolContact(direction)
            current_pose = rtde_r.getActualTCPPose()
            distance = start_pose[2] - current_pose[2]
            if flag_contact > 0:
                rtde_c.stopL(1.5)
                flag = True
            if distance > 0.065:
                flag = True
    
    # Open gripper
    rtde_c.gripperDisable()
    rtde_c.gripperRelease(0)
    time.sleep(1.5)

    pose = rtde_r.getActualTCPPose()
    pose[2] = pose[2] + 0.2
    rtde_c.moveL(pose, 0.2, 0.2)
    rtde_c.moveJ(take_image_pose_joints, 0.8, 0.8)
    # Place component
    #place_component(rtde_c=rtde_c, rtde_r=rtde_r, placing_pose = placing_pose, fix_center = fix_center)

    rtde_c.gripperDisable()
    rtde_c.stopScript()
    rtde_c.disconnect()

    quit()
    rospy.spin()