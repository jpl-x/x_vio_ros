#!/bin/bash

if [ -n "$1" ]
then
T=$1
else
T=5
fi

echo " ### plotting position variances x y z ###"
echo " ### buffer = " $T "sec"

rqt_plot /x_vio/cov_core_imu_rate/data[0]:data[16]:data[32]
