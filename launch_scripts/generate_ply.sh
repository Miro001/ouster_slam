#!/bin/bash
source ../../devel/setup.bash
bag_filenames="/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo_long/carto_slam_2020-04-07-10-17-45_0.bag /mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo_long/carto_slam_2020-04-07-10-20-13_1.bag /mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo_long/carto_slam_2020-04-07-10-22-26_2.bag"
pbstream_filename="/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo_long/carto_slam_2020-04-07-10-17-45.bag.pbstream"
urdf_filename="/home/miro/Documents/ros/oslam_ws/src/ouster_slam/urdf/os_sensor.urdf" #own path

roslaunch ouster_slam assets_writer_ros_map.launch \
   bag_filenames:="${bag_filenames}" \
   pose_graph_filename:=${pbstream_filename}\
   urdf_filename:=${urdf_filename}