# ReFeree radar SLAM, ROS 1 Noetic: Yeti radar odometry + ReFeree loop closing + iSAM2 (the former
# ReFeree-Radar-SLAM), built against the C++ core of this repository.
#   docker build -f docker/ros1/Dockerfile -t referee:noetic .
#   docker run --rm -v /path/to/MulRan/KAIST_03:/data/KAIST_03:ro -v $PWD/output:/data/output referee:noetic \
#     roslaunch referee_radar_slam referee_radar_slam.launch seq_dir:=/data/KAIST_03/ save_directory:=/data/output/
FROM osrf/ros:noetic-desktop
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git libeigen3-dev libopencv-dev libpcl-dev libboost-all-dev libomp-dev \
        python3-catkin-tools ros-noetic-gtsam ros-noetic-pcl-ros ros-noetic-cv-bridge ros-noetic-image-transport \
    && rm -rf /var/lib/apt/lists/*
COPY cpp /referee/cpp
COPY applications/slam/ros1 /referee/applications/slam/ros1
WORKDIR /referee/ws
RUN source /opt/ros/noetic/setup.bash && catkin init && \
    catkin config --source-space /referee/applications/slam/ros1 --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build -j8 --no-status nano_gicp && catkin build -j8 --no-status
RUN echo "source /referee/ws/devel/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/bin/bash", "-c", "source /referee/ws/devel/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
