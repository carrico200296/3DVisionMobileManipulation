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


if __name__ == "__main__":
 
    rospy.init_node('manipulation_node')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    nb_components = rospy.wait_for_message("/pose_estimation/nb_components", std_msgs.msg.String, rospy.Duration(300.0))
    rtde_c = rtde_control.RTDEControlInterface("192.168.10.20")
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.10.20")

    base_frame = "base"
    print(":: Nunber of components detected = %d" %(int(nb_components.data)))

    for i in range(int(nb_components.data)):
        component_frame = "object_frame_" + str(i+1)
        tf_transform_base_component = tfBuffer.lookup_transform(base_frame, component_frame, rospy.Time(0))

        t = [tf_transform_base_component.transform.translation.x,
             tf_transform_base_component.transform.translation.y,
             tf_transform_base_component.transform.translation.z]

        quaternion = [tf_transform_base_component.transform.rotation.x,
                      tf_transform_base_component.transform.rotation.y,
                      tf_transform_base_component.transform.rotation.z,
                      tf_transform_base_component.transform.rotation.w]

        print(":: Pose Component %d" %(i+1))
        print("Translation:")
        print("   x: %.4f mm" %(t[0]*1000))
        print("   y: %.4f mm" %(t[1]*1000))
        print("   z: %.4f mm" %(t[2]*1000))
        '''
        rot_mtx = R.from_quat(quaternion).as_dcm()
        rot_x = R.from_euler('y', -90, degrees=True).as_dcm()
        rot = R.from_dcm(np.linalg.multi_dot([rot_x, rot_mtx])).as_rotvec()
        print("   Rotation Axis with rotation Y axis -90 degrees:")
        print("   Rx: %.4f rad" %rot[0])
        print("   Ry: %.4f rad" %rot[1])
        print("   Rz: %.4f rad" %rot[2])

        rot_mtx = R.from_quat(quaternion).as_dcm()
        rot_x = R.from_euler('y', 90, degrees=True).as_dcm()
        rot = R.from_dcm(np.linalg.multi_dot([rot_x, rot_mtx])).as_rotvec()
        print("   Rotation Axis with rotation Y axis 90 degrees:")
        print("   Rx: %.4f rad" %rot[0])
        print("   Ry: %.4f rad" %rot[1])
        print("   Rz: %.4f rad" %rot[2])
        '''
        rot = R.from_quat(quaternion).as_rotvec()
        print("   Rotation Axis with NO rotation:")
        print("   Rx: %.4f rad" %rot[0])
        print("   Ry: %.4f rad" %rot[1])
        print("   Rz: %.4f rad" %rot[2])

        #rot = [2.134, -2.338, -0.102]
        tool_offset = 0.1245
        off_set = tool_offset + 0.07 

        rtde_c.moveL([t[0], t[1], t[2] + off_set, rot[0], rot[1], 0.0], 0.05, 0.2)
        current_TCP = rtde_r.getActualTCPPose()
        picking_pose = current_TCP[:]
        picking_pose[2] = 0.1315
        rtde_c.moveL(picking_pose, 0.05, 0.1)
        rtde_c.gripperDisable()
        time.sleep(0.2)
        rtde_c.gripperGrasp(0)
        time.sleep(1)
        current_TCP[2] = current_TCP[2] + off_set
        rtde_c.moveL(current_TCP, 0.05, 0.1)
        rtde_c.moveL([0.41022, -0.3445, 0.22424, 2.14, -2.301, 0.0], 0.1, 0.1)
        time.sleep(0.3)
        rtde_c.gripperRelease(0)
        time.sleep(3.0)
        rtde_c.moveL(current_TCP, 0.05, 0.1)

    rtde_c.stopScript()
    rtde_c.disconnect()

    quit()
    rospy.spin()