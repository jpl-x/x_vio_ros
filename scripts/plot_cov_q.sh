#!/bin/bash

if [ -n "$1" ]
then
T=$1
else
T=5
fi

echo " ### plotting attitude variances thetax thetay thetaz ###"
echo " ### buffer = " $T "sec"

rqt_plot /x_vio/cov_core_imu_rate/data[96]:data[112]:data[128]
