#!/bin/bash
gnome-terminal -- bash -c "cd /home/$1;source /opt/ros/melodic/setup.bash;rosbag play $2"&
#source /opt/ros/melodic/setup.bash
#rosbag play /home/$1/$2
#echo "str"
exit 0
