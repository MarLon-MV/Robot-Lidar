#!/usr/bin/env bash
# Environment for the MTRB workspace (Pi 5 and any ROS 2 Jazzy machine on the robot network).
# Usage:  source env.sh

# Same DDS settings as the Create 3 (domain 21, CycloneDDS).
export ROS_DOMAIN_ID=21
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

_WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${_WS_DIR}/install/setup.bash" ]; then
  source "${_WS_DIR}/install/setup.bash"
fi
unset _WS_DIR
