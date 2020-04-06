#!/bin/bash
source ../../devel/setup.bash

BAGPATH="/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/bags/carto_slam_2020-04-06-15-56-15_0.bag /mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/bags/carto_slam_2020-04-06-16-00-58_2.bag /mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/bags/carto_slam_2020-04-06-15-58-46_1.bag"
CONFIGPATH='/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo/config.json' #own path

CARTOCONF='/home/miro/Documents/ros/oslam_ws/src/ouster_slam/' #own path

roslaunch ouster_slam os_slam.launch \
                             udp_hostname:=127.0.0.1 \
                             replay:=true \
                             bag_filename:="${BAGPATH}"\
                             metadata:=${CONFIGPATH}\
                             carto_conf:=${CARTOCONF}\
                             log_landmarks_path:='/home/miro/Desktop/log.txt'