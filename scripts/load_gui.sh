#! /bin/bash
# This script loads and stops the xVIO GUI.

# Load GUI
rqt --clear-config --perspective-file $(rospack find x_vio_ros)/cfg/rqt.perspective &
rviz -d $(rospack find x_vio_ros)/cfg/config.rviz &
sleep 1 # Ensure user prompt below is the last thing to show up on screen

# Wait for user to press q to kill the script
while [ -z $key_pressed ] || [ $key_pressed != 'q' ]; do
read -p 'Press q to kill the GUI: ' key_pressed
done

# Kill GUI
killall -9 rqt
killall -9 rviz
