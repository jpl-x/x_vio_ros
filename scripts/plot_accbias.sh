#!/bin/bash

if [ -n "$1" ]
then
T=$1
else
T=5
fi

echo " ### plotting acceleration bias x y z [m/s^2] ###"
echo " ### buffer = " $T "sec"

rqt_plot /x_vio/state_imu_rate/data[13]:data[14]:data[15] 
