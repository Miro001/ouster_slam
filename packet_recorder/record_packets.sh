#!/bin/bash
source ../../devel/setup.bash

BAGFILEPATH='/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo/stvorec_proti_smeru_hr_2.bag' #own path
METAFILEPATH='/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/geo/config.json' #own path

RECORDFILEPATH='/mnt/b6ef98c1-c7b0-4b69-87d0-2165d664c748/bags/debug/';

roslaunch packet_recorder os1_packet_record.launch\
                             udp_hostname:=127.0.0.1\
                             replay:=true \
                             bag_filename:=${BAGFILEPATH}\
                             metadata:=${METAFILEPATH}\
                             record:=true\
                             record_filepath:=${RECORDFILEPATH}