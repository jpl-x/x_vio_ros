#!/bin/bash

if [ -n "$1" ]
then
T=$1
else
T=5
fi

echo " ### plotting velocities x y z [m/s] ###"
echo " ### buffer = " $T "sec"

rqt_plot /x_vio/state_imu_rate/data[3]:data[4]:data[5]
