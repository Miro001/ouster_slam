#!/bin/bash
source ../../devel/setup.bash

bags="/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/debug/all_slam_2020-04-06-12-07-31_0.bag /mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/debug/all_slam_2020-04-06-12-07-57_1.bag"

roslaunch landmark_publisher landmark_node_glob_debug.launch \
bag_filename:="${bags}"


