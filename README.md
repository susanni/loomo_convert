# Re-publishing Loomo topics for FASTER mapper #

### Example sequence of commands to run with [FASTER](https://github.com/mit-acl/faster):
```bash
rosbag play --clock 2020-03-10-18-47-09.bag /LO01/realsense_loomo/depth/camera_info:=/LO01/realsense_loomo/camera_info
roslaunch loomo_convert convert.launch
roslaunch global_mapper_ros global_mapper_node.launch quad:=LO01 depth_image_topic:=realsense_loomo/depth_new_encoding pose_topic:=state goal_topic:=move_base_simple/goal odom_topic:=odom

roslaunch faster faster_interface.launch quad:= LO01 is_ground_robot:=true
roslaunch faster faster.launch quad:=LO01
```

Notes:
* FASTER mapper expects image topics of format `<camera_name>/<depth_image_topic_name>` and `<camera_name>/camera_info`, which is why `camera_info` topic was remapped when the bag file was played
* `depth_image_topic` expects `sensor_msgs/Image` message, `pose_topic` expects `snapstack_msgs/State`, and `odom_topic` expects `nav_msgs/Odometry`, which was not directly provided from Loomo topics.

### Lines that have to be changed in FASTER repo:
* In `acl-mapping/global-mapper/global_mapper_ros/launch/global_mapper_node.launch`
  * If running mapper with bag file of Loomo data, add line:
    ```xml
      <param name="/use_sim_time" value="true" />
    ```
* In `acl-mapping/global-mapper/global_mapper_ros/src/global_mapper_ros/global_mapper_ros.cc`
  * If depth images from Loomo are of `mono16` encoding, change line 598 to:
    ```cpp
      if (image_msg->encoding == "16UC1" || image_msg->encoding == "mono16")
    ```
  * If camera sets out-of-range values to 0 instead of NaN or Inf, in `DepthImageCallback` (around line 646) add:
    ```cpp
      else if (depth == 0) { continue; }  // don't add points to pointcloud if value/depth is 0
    ```
    

