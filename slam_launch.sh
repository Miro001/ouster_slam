#!/bin/bash
source ../devel/setup.bash

BAGPATH='/media/miro/DataStore/bags/geo/stvorec_proti_smeru_hr.bag' #own path
CONFIGPATH='/media/miro/DataStore/bags/geo/config.json' #own path

CARTOCONF='/home/miro/Documents/ros/ouster_slam_ws/src/ouster_slam/' #own path

roslaunch ouster_slam os_slam.launch \
                             udp_hostname:=127.0.0.1 \
                             replay:=true \
                             bag_filename:=${BAGPATH}\
                             metadata:=${CONFIGPATH}\
                             carto_conf:=${CARTOCONF}