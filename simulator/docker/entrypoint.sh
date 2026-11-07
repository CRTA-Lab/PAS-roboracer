#!/bin/bash
# Source ROS and the prebuilt simulator workspace, then run the given command.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/sim_ws/install/setup.bash
exec "$@"
