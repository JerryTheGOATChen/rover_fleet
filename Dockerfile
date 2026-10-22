# The whole fleet stack in one image: ROS 2 Jazzy, Gazebo Harmonic, Nav2,
# slam_toolbox, the rover packages, and CPU-only PyTorch for training.
# CI builds this image and runs the tests inside it.
#
#   docker build -t rover-fleet .
#   docker run --rm rover-fleet bash -lc "cd /ws && colcon test && colcon test-result --verbose"
#   docker run --rm -it rover-fleet bash -lc "cd /ws/src/fleet_rl && python3 -m fleet_rl.evaluate --only main"
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
WORKDIR /ws

# Dependencies first, so they stay cached until a package.xml changes.
COPY fleet_rl/package.xml src/fleet_rl/package.xml
COPY rover_fleet/package.xml src/rover_fleet/package.xml
COPY rover_autonomy/package.xml src/rover_autonomy/package.xml
COPY rover_bringup/package.xml src/rover_bringup/package.xml
COPY rover_description/package.xml src/rover_description/package.xml
RUN apt-get update \
 && rosdep update --rosdistro jazzy \
 && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
 && apt-get install -y --no-install-recommends python3-pip python3-pytest python3-pyflakes \
 && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir --break-system-packages torch \
      --index-url https://download.pytorch.org/whl/cpu

COPY fleet_rl src/fleet_rl
COPY rover_fleet src/rover_fleet
COPY rover_autonomy src/rover_autonomy
COPY rover_bringup src/rover_bringup
COPY rover_description src/rover_description
RUN source /opt/ros/jazzy/setup.bash && colcon build --event-handlers console_cohesion+ \
 && echo "source /ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
