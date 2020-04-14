#!/bin/bash
source ../../devel/setup.bash
base_path="/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/";

bag_filenames="${base_path}geo_long/map_gen_0.bag"
pbstream_filename="${base_path}/geo_long/map_gen.bag.pbstream"

rosservice call /finish_trajectory 0

rosservice call /write_state "{filename: "${pbstream_filename}"  }"

roslaunch ouster_slam assets_writer_ros_map.launch \
   bag_filenames:="${bag_filenames}" \
   pose_graph_filename:=${pbstream_filename}