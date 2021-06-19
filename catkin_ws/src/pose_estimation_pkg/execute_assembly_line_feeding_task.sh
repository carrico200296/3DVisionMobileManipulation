#!/usr/bin/env bash
#cd /home/ccdn/catkin_ws/src/countingcomponents/src/scripts/
#echo "Connecting the camera...."

gnome-terminal -e "roslaunch pose_estimation_pkg rs_stream_depth_tf_tree.launch"
sleep 8
gnome-terminal -e "roslaunch pose_estimation_pkg test_files_rviz.launch"