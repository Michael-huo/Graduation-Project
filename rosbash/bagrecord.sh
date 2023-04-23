#!/bin/bash
cd /home/$1/$2
bagname=$3
echo "记录的包名：$bagname "
source /opt/ros/melodic/setup.bash
rosbag record -a
sleep 5
