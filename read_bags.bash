#!/bin/bash

if [ -z "$1" ]
  then
    echo "**************************************************************"
    echo "No folder location is given"
    exit 1
fi

source 'catkin_ws/devel/setup.bash'

folder_loc="$1/*"
for file in $folder_loc
do
    echo "____________________________________________________"
    echo "playing:$file"
    rosbag play $file
done