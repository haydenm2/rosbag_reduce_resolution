ROSbag Resolution Reducer
==============================

This ROS package, known as `rosbag_reduce_resolution`, includes both a launch approach and a python parsing approach for taking in a target ROSbag and outputting a bag containing the designated video topic at a specified resolution reduction ratio. The python parser has been shown to work most effectively as the launch method may drop frames depending on available system resources.

## Setup
Create a catkin workspace (e.g. rosbag_ws)
```bash
$ mkdir rosbag_ws
```
Create a src folder in new workspace
```bash
$ cd rosbag_ws
$ mkdir src
```
Clone this git repository into the src folder
```bash
$ cd src
$ git clone https://github.com/haydenm2/rosbag_reduce_resolution.git
```
Clone the fizyr-forks vision_opencv from https://github.com/fizyr-forks/vision_opencv.git into the src folder
```bash
$ cd src
$ git clone https://github.com/fizyr-forks/vision_opencv.git
```
In the catkin workspace run the catkin make command from the workspace folder (e.g. rosbag_ws) with the following flags (replacing "python3.8" with the applicable python 3 version installed on your system)
```bash
$ cd ..
$ catkin_make -DPYTHON_EXECUTABLE=$(which python3) -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DPYTHON_LIBRARY=/usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so
```
Then source the workspace
```bash
$ source devel/setup.bash
```

## Reduction Through Python Parsing (Fastest Option)
The python parsing script directly parses a given ROSbag file on a specified image topic. It can be run with the following command:

```bash
$ cd /<path_to_workspace>/catkin_ws/src/rosbag_reduce/scripts
$ python parse_reduce.py -o '/<desired_output_rosbag_path>/<output_rosbag_name>.bag' -t '<video_rostopic_name>' -i '/<path_to_input_rosbag>/<input_rosbag_name>.bag' -r <resolution_resize_ratio> -p <passthrough_topics (optional)> -c <camera_info_topic (optional)>
```
In general the following flags indicate:
`-o` = Output ROSbag path
`-i` = Input ROSbag path
`-t` = Input ROSbag image topic to reduce
`-r` = Resolution resize ratio (e.g. -r 0.5 will result an output image of half the pixels on each axis. Thus, a 4000x3000 image will be reduced to 2000x1500).
`-p` = Input ROSbag topics to pass through to the next ROSbag (must be seperated by spaces) (e.g. /gps /imu /gpstime)
`-c` = ROSbag camera info topic to adjust with reduction

## Reduction Through ROSlaunch (Slower Option)
The roslaunch reduction method plays the desired input rosbag on a ROS network, reduces published images on the designated image topic, publishes the reduced images on a new topic, and records the reduced image into a new ROSbag. This method has a tendency to drop frames if system resources are overused due to other background tasks or very high-resolution images. It can be run with the following command:

```bash
$ cd /<path_to_workspace>
$ source devel/setup.bash
$ roslaunch rosbag_reduce rosbag_reduce.launch bag_path:="<path_to_input_bag>/<input_bag_name>.bag" bag_topic:="<input_rostopic_name>" reduced_bag_path:="/<desired_output_rosbag_path>/<output_rosbag_name>.bag" reduced_bag_topic:="<output_rostopic_name>"

```
Note: Unfortunately due to the nature of this roslaunch approach, the reduced_bag_topic parameter CANNOT be the same as the input bag_topic. Doing so duplicates publication onto the same topic and results in duplicated images with mixed frame resolutions.




