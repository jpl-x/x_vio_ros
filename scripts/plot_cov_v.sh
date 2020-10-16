#!/bin/bash

if [ -n "$1" ]
then
T=$1
else
T=5
fi

echo " ### plotting velocity variances vx vy vz ###"
echo " ### buffer = " $T "sec"

rqt_plot /x_vio/cov_core_imu_rate/data[48]:data[64]:data[80]
