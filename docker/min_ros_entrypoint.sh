#!/bin/bash
set -e

# setup ROS2 environment
source /opt/minimal_effort/setup.bash
exec "$@"
