#!/bin/bash
set -e

# Set up ROS environment
#source /opt/ros/kinetic/setup.bash

# Single machine configuration
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

exec "$@"
