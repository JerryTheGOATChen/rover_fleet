#!/bin/bash
# Inside the container: virtual display -> RViz -> ffmpeg screen capture to /out/$1.mp4 for $2 seconds.
set -e
source /opt/ros/jazzy/setup.bash
Xvfb :99 -screen 0 1280x720x24 -nolisten tcp >/out/xvfb.log 2>&1 &
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1
sleep 2
rviz2 -d /cfg/record.rviz --ros-args -p use_sim_time:=true >/out/rviz.log 2>&1 &
sleep 10
ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -video_size 1280x720 -framerate 10 -i :99 -t "$2" \
       -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p "/out/$1.mp4"
