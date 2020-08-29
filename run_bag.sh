#!/usr/bin/env bash
#Currently ros2 bag doesn't have loop functionality, so using this script instead
while true; do ros2 bag play ./data/bag/rosbag2_2020_08_27-10_07_41; done
