#!/bin/bash

if [ -n "$1" ]
then
T=$1
else
T=5
fi

echo " ### plotting position x y z [m] ###"
echo " ### buffer = " $T "sec"

rqt_plot /x_vio/state_imu_rate/data[0]:data[1]:data[2]
