#!/bin/bash

if [ -n "$1" ]
then
T=$1
else
T=5
fi

echo " ### plotting gyroscope variances bwx bwy bwz ###"
echo " ### buffer = " $T "sec"

rqt_plot /x_vio/cov_core_imu_rate/data[144]:data[160]:data[176]
