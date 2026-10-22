# Off-screen RViz recorder (tools/rviz_recorder): joins the host ROS graph (--net=host)
# and films RViz on a virtual display, so no window or GPU is needed.
FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rviz2 ros-jazzy-rviz-default-plugins xvfb ffmpeg fonts-dejavu-core \
 && rm -rf /var/lib/apt/lists/*
