#!/bin/bash
pkill -9 -f __name
pkill -9 -f rosmaster
pkill -9 -f openhrp-model-loader

sleep 3

roslaunch hrpsys_ros_bridge_jvrc jaxon_jvrc_choreonoid.launch > /dev/null
