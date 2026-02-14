FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-tf-transformations \
    ros-humble-rviz2 \
    xvfb \
    x11vnc \
    fluxbox \
    novnc \
    websockify \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

WORKDIR /ws/src/coverage_control

COPY requirements/lib.txt /tmp/requirements-lib.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements-lib.txt
COPY docker/run_with_novnc.sh /usr/local/bin/run_with_novnc.sh
RUN chmod +x /usr/local/bin/run_with_novnc.sh

WORKDIR /ws

CMD ["/bin/bash"]
