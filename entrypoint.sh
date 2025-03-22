#!/bin/bash

set -e

cd ~/DNRS_assignment_2/ros2_ws
colcon build

source install/local_setup.bash

exec $@
