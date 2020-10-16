#!/bin/bash

if [ -n "$1" ]
then
T=$1
else
T=5
fi

echo " ### plotting gyro bias x y z [rad/s] ###"
echo " ### buffer = " $T "sec"

rqt_plot /x_vio/state_imu_rate/data[10]:data[11]:data[12]
