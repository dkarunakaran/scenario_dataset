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
    rostopic pub /name_of_the_file std_msgs/String "$file" --once
    echo "published rostopic /name_of_the_file"
    rosbag play $file
    rostopic pub /save_file std_msgs/String "" --once
    echo "published rostopic /save_file"
done