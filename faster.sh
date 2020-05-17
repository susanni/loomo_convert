#!/bin/bash
# Run FASTER for Loomo

roslaunch loomo_convert convert.launch > /dev/null 2>&1 &

roslaunch global_mapper_ros global_mapper_node.launch quad:=LO01 depth_image_topic:=realsense_loomo/depth_new_encoding pose_topic:=state goal_topic:=move_base_simple/goal odom_topic:=odom > /dev/null 2>&1 &

roslaunch faster faster.launch quad:=LO01 &

roslaunch faster faster_interface.launch quad:=LO01 is_ground_robot:=true
