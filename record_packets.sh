#!/bin/bash
source ../devel/setup.bash

BAGFILEPATH='/media/miro/DataStore/bags/geo/stvorec_proti_smeru_hr_2.bag' #own path
METAFILEPATH='/media/miro/DataStore/bags/geo/config.json' #own path
RECORDFILEPATH='/media/miro/DataStore/bags/geo/' #own path

roslaunch ouster_slam os1_packet_record.launch \
                             udp_hostname:=127.0.0.1\
                             replay:=true \
                             bag_filename:=${BAGFILEPATH}\
                             metadata:=${METAFILEPATH}\
                             record:=true\
                             record_filepath:=${RECORDFILEPATH}