#!/bin/bash

echo " ### plotting sliding-window attitude variances x y z ###"

if [ $# -ne 2 ]
then
	echo "Usage: ./plot_poses_att arg1 arg2
	arg1: # Maximum number of poses in the sliding window
	arg2: Maximum number of persistent features"
	exit
fi

# Maximum number of pose in the sliding window
n_poses=$1

# Maximum number of persistent features
n_features=$2

# Compute dimension of the auxilliary covariance matrix
# (calib+poses+features covariance matrix)
let "n_aux = 6 + 6 * n_poses + 3 * n_features"

# Initialize the plot command
com='rqt_plot /x_vio/cov_aux_imu_rate/'

# Maximum loop index
let "idx_max = 3 * n_poses - 1"

# Building iteratively the plot command
for i in `seq 0 $idx_max`;
do
	# Index of current sliding window position variance
	let "idx = (6 + 3 * n_poses + i) * (n_aux + 1)"
	
 	# Command build-up
	add=data[$idx]:
	com=$com$add
done

# Remove last colon
com=${com::-1}

# Run command
`$com`
