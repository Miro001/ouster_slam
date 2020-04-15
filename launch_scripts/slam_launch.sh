#!/bin/bash
source ../../devel/setup.bash
base_path="/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/";

bag_filenames="${base_path}geo_long/carto_slam_2020-04-07-10-17-45_0.bag /mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo_long/carto_slam_2020-04-07-10-20-13_1.bag /mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo_long/carto_slam_2020-04-07-10-22-26_2.bag"
config_path="${base_path}geo_long/caconfig.json" #own path
record_filepath="${base_path}geo_long/";

roslaunch ouster_slam os_slam.launch \
                             udp_hostname:=127.0.0.1 \
                             replay:=true \
                             bag_filename:="${bag_filenames}"\
                             metadata:=${config_path}