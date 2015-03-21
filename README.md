# tag_filter
ROS node for cropbox filter based on detected AR tags

Required Packages
-----------------------
[ar_track_alvar](http://wiki.ros.org/ar_track_alvar) and ar_track_alvar_msgs are required to build this package. These can be simple installed using apt-get as
`sudo apt-get install ros-indigo-ar-track-alvar*`

Usage
----------------------
After getting four tags from the above mentioned link, we can use them directly using the node `dynamic_cropbox` - provided a cropbox filter is already running.
A demo application using an ASUS connected, can be run using:
`roslaunch tag_filter launch_tag.launch`

This application initializes a cropbox filter nodelet, runs a node that subscribes to the cropbox filter output, while dynamically monitoring the tags to maintain the cropbox around the area of interest. In case the ASUS creates problems during the launch, refer to [pcl_495](https://github.com/ritwik1993/pcl_495). 
