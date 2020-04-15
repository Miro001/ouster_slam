#!/bin/bash
source ../../devel/setup.bash

bag_filenames="/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo_long/carto_slam_2020-04-07-10-17-45_0.bag /mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo_long/carto_slam_2020-04-07-10-20-13_1.bag /mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo_long/carto_slam_2020-04-07-10-22-26_2.bag"
config_path='/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo_long/caconfig.json' #own path

roslaunch packet_recorder os1_packet_record.launch \
                             udp_hostname:=127.0.0.1 \
                             replay:=true \
                             bag_filename:="${bag_filenames}"\
                             metadata:=${config_path}\
                             record:=false\
                             viz:=true