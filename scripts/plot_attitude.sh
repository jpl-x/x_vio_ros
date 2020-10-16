#!/bin/bash

if [ -n "$1" ]
then
T=$1
else
T=5
fi

echo " ### plotting attitude qw qx qy qz [quat] ###"
echo " ### buffer = " $T "sec"

rqt_plot /x_vio/state_imu_rate/data[6]:data[7]:data[8]:data[9]
