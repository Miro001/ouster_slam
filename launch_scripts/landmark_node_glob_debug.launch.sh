#!/bin/bash
source ../../devel/setup.bash

bag_filenames="/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo_long/carto_slam_2020-04-07-10-17-45_0.bag /mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo_long/carto_slam_2020-04-07-10-20-13_1.bag /mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo_long/carto_slam_2020-04-07-10-22-26_2.bag"

roslaunch landmark_publisher landmark_node_glob_debug.launch \
                              bag_filename:="${bag_filenames}"


