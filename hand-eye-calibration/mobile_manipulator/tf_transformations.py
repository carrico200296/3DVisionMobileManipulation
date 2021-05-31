#! /usr/bin/env python

# This script is used to get connect the /camera_link with the /wrist_3_link, once we know
# the transformation from camera_color_optical_frame to wrist_3_link

import numpy as np
import scipy
from scipy.spatial.transform import Rotation as R

# TF TRANSFORMATIONS for Realsense D415 and URe robot in the mobile manipulator
# info: RPY (roll pitch yaw) is rotation around ZYX

'''
$ rosrun tf tf_echo camera_link camera_color_optical_frame -->  # transformation of camera_color_optical_frame wrt camera_link
- Translation: [0.000, 0.015, 0.000]
- Rotation: in Quaternion [-0.498, 0.499, -0.501, 0.501]
            in RPY (radian) [-1.565, 0.001, -1.572]
            in RPY (degree) [-89.688, 0.061, -90.072]
'''
T_cameraLink_colorOptical = np.eye(4)
trans_cameraLink_colorOptical = [0.000, 0.015, 0.000]
R_cameraLink_colorOptical = R.from_quat([-0.498, 0.499, -0.501, 0.501]).as_dcm()
T_cameraLink_colorOptical[:3,:3] = R_cameraLink_colorOptical 
T_cameraLink_colorOptical[:3,3] = trans_cameraLink_colorOptical 

'''
$ rosrun tf tf_echo camera_color_optical_frame camera_link
- Translation: [0.015, 0.000, -0.000]
- Rotation: in Quaternion [0.498, -0.499, 0.501, 0.501]
            in RPY (radian) [-0.227, -1.565, 1.797]
            in RPY (degree) [-13.030, -89.680, 102.970]
'''
T_colorOptical_cameraLink = np.eye(4)
trans_colorOptical_cameraLink = [0.015, 0.000, 0.000]
R_colorOptical_cameraLink = R.from_quat([0.498, -0.499, 0.501, 0.501]).as_dcm()
T_colorOptical_cameraLink[:3,:3] = R_colorOptical_cameraLink
T_colorOptical_cameraLink[:3,3] = trans_colorOptical_cameraLink
#R_colorOptical_cameraLink = np.linalg.inv(R_cameraLink_colorOptical) same approach


'''
$ rosrun tf tf_echo wrist_3_link camera_color_optical_frame  
from calibration (camera_pose.launch): xyz = [-0.0323172 -0.0837986 0.0145889]
                                       quaternion = [0.00641723 -0.00484419 0.00206152 0.999966]
'''
T_wrist_colorOptical = np.eye(4)
trans_wrist_colorOptical = [-0.0323172, -0.0837986, 0.0145889]
R_wrist_colorOptical = R.from_quat([0.00641723, -0.00484419, 0.00206152, 0.999966]).as_dcm()
T_wrist_colorOptical[:3,:3] = R_wrist_colorOptical
T_wrist_colorOptical[:3,3] = trans_wrist_colorOptical

# Transformation of /camera_link frame respect to the /wrist_3_link frame
# T_wrist_cameraLink = T_wrist_colorOptical x T_colorOptical_cameraLink
T_wrist_cameraLink = np.linalg.multi_dot([T_wrist_colorOptical, T_colorOptical_cameraLink])
trans_wrist_cameraLink = T_wrist_cameraLink[:3,3]
q_wrist_cameraLink = R.from_dcm(T_wrist_cameraLink[:3,:3]).as_quat()

print("Homogeneous Transformation of /camera_link respect to /wrist_3_frame:")
print(T_wrist_cameraLink)
print("Translation: X Y Z")
print(trans_wrist_cameraLink)
print("Rotation: quaternion")
print(q_wrist_cameraLink)
print("Include these vaules in the launch files of pose_estimation_pkg")