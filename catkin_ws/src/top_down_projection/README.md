# Package: top_down_projection

## Description:
Projects the lidar data from a bag file into an image.

### top_down_projection

**Description** : Projects the lidar information into a 2d grid which is output to an image file

**Parameters**

* point_clouds: an array containing elements to draw, each element is a comma separated string
```
format: "topic_name,draw_type,alpha_value,draw_point_size"
draw type: circle, point
eg: "/velodyne/front/points,circle,10,6" will draw a circle for each of the points in the topic /velodyne/front/points size 6 alpha value of 10
```

* projection_frame: frame to apply the top down projection - default is "odom"
* bag_file: bag file for input data
* output_image: output image file (can be any opencv type)

* min_intensity, max_intensity: range of lidar intensity for colourising the projection - different lidars have different ranges of reported intensity

* parameters for the h264 bag playback - some useful parameters listed below, for more parameters see h264_bag_playback project
  * percentage_start: place in the bag file to start
  * percentage_end: place in the bag file to end
  * horizon_in_buffer: set value="true" to enable the correction of the moving platform using the IMU

* use_rings: an array of numbers for the lidar ring numbers to use in the projection

**Author** : Stewart Worrall

*Note:* The intensity values for the ibeo data is in the range of approximately 0 to 5, the velodyne data is in the range 0 to 255
*Note:* If you press ctrl c when it is running, it will make the map for the points that have been read so far

See the default launch file project_lidar.launch for an example