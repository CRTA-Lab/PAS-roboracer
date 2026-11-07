#!/bin/bash
# First start: copy the lab's example into /lab_ws/src (the student's mounted
# folder) and build it. Later starts reuse whatever the student has there.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"

mkdir -p /lab_ws/src
if [ -z "$(ls -A /lab_ws/src)" ]; then
    echo "[lab] Copying the ${LAB} example into /lab_ws/src"
    cp -a /opt/lab_ws/src/. /lab_ws/src/
fi

if [ ! -f /lab_ws/install/setup.bash ]; then
    echo "[lab] Building /lab_ws"
    (cd /lab_ws && colcon build --symlink-install)
fi

# Files created by root in a bind mount would be root-owned on a Linux host;
# hand them to whoever owns the mounted folder.
chown -R "$(stat -c %u:%g /lab_ws)" /lab_ws

source /lab_ws/install/setup.bash
exec "$@"
