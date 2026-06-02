#!/bin/sh

. /opt/ros/jazzy/setup.sh
[ "$1" != "--no-build" ] && colcon build --symlink-install
[ -e install/setup.sh ] && . install/setup.sh
