#!/bin/bash

if [ -n "$1" ]
then
T=$1
else
T=5
fi

echo " ### plotting position x y z [m] ###"
echo " ### plotting velocities x y z [m/s] ###"
echo " ### plotting attitude qw qx qy qz [quat] ###"
echo " ### plotting gyroscope bias x y z [rad/s] ###"
echo " ### plotting accelerometer bias x y z [m/s^2] ###"
echo " ### buffer = " $T "sec"

rqt_plot /x_vio/state_imu_rate/data[0]:data[1]:data[2] &
rqt_plot /x_vio/state_imu_rate/data[3]:data[4]:data[5] &
rqt_plot /x_vio/state_imu_rate/data[6]:data[7]:data[8]:data[9] &
rqt_plot /x_vio/state_imu_rate/data[10]:data[11]:data[12] &
rqt_plot /x_vio/state_imu_rate/data[13]:data[14]:data[15]
