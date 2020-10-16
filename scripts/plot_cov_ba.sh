#!/bin/bash

if [ -n "$1" ]
then
T=$1
else
T=5
fi

echo " ### plotting accelerometer variances bax bay baz ###"
echo " ### buffer = " $T "sec"

rqt_plot /x_vio/cov_core_imu_rate/data[192]:data[208]:data[224]
