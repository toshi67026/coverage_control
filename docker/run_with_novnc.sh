#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

# Virtual desktop for rviz2 on headless Docker.
export DISPLAY=:1
export NO_AT_BRIDGE=1

Xvfb :1 -screen 0 1920x1080x24 &
fluxbox >/tmp/fluxbox.log 2>&1 &
x11vnc -display :1 -nopw -forever -shared -rfbport 5900 >/tmp/x11vnc.log 2>&1 &
/usr/share/novnc/utils/launch.sh --vnc localhost:5900 --listen 6080 >/tmp/novnc.log 2>&1 &

rosdep update
rosdep install -i -y --from-paths /ws/src/coverage_control/coverage_control --rosdistro humble
colcon build --symlink-install --base-paths /ws/src/coverage_control/coverage_control
source /ws/install/setup.bash

exec ros2 launch coverage_control scc.launch.py dim:=2 num:=3 phi:=1 rviz:=true
