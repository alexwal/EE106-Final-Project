#!/usr/bin/env bash

export MACHINE_TYPE=zumy
export ZUMY_WORKSPACE=/home/$MACHINE_TYPE/ros_zumy

source $ZUMY_WORKSPACE/devel/setup.bash
export ROS_HOSTNAME=$HOSTNAME.local
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ZUMY_WORKSPACE
exec "$@"
