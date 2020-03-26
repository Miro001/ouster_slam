#!/bin/bash
source ../devel/setup.bash

BAGPATH='/media/miro/DataStore/bags/geo/stvorec_proti_smeru_hr_2.bag' #own path
CONFIGPATH='/media/miro/DataStore/bags/geo/config.json' #own path

#BAGPATH='/media/miro/DataStore/bags/geo/stvorec_proti_smeru_hr.bag' #own path
#CONFIGPATH='/media/miro/DataStore/bags/geo/config.json' #own path

roslaunch ouster_slam os1_from_bag.launch \
                             udp_hostname:=127.0.0.1 \
                             replay:=true \
                             bag_filename:=${BAGPATH}\
                             metadata:=${CONFIGPATH}\
                             record:=true