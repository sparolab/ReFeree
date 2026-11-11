# ReFeree radar SLAM, ROS 2 Humble: the C++ core + ReFeree odometry + pose-graph SLAM (GTSAM 4.2).
#   docker build -f docker/ros2/Dockerfile -t referee:humble .
#   docker run --rm -v /path/to/MulRan/KAIST_03:/data/KAIST_03:ro -v $PWD/output:/data/output referee:humble \
#     ros2 launch referee_slam referee_slam.launch.py sequence_dir:=/data/KAIST_03 save_directory:=/data/output/
FROM ros:humble-ros-base-jammy
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git libboost-all-dev libeigen3-dev libopencv-dev libfftw3-dev libtbb-dev libomp-dev \
        python3-pip python3-numpy python3-scipy python3-opencv python3-pytest python3-colcon-common-extensions \
        ros-humble-cv-bridge ros-humble-image-transport ros-humble-pcl-ros ros-humble-pcl-conversions \
        ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*
# GTSAM 4.2, C++17, system Eigen so it agrees with PCL and the ROS stack
RUN git clone --depth 1 --branch 4.2 https://github.com/borglab/gtsam.git /opt/gtsam && \
    cmake -S /opt/gtsam -B /opt/gtsam/build -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /opt/gtsam/build -j8 --target install && ldconfig && rm -rf /opt/gtsam

COPY cpp /referee/cpp
COPY python /referee/python
COPY applications/slam/ros2 /referee/applications/slam/ros2
COPY docker/validate.sh /referee/docker/validate.sh
# the core and its checks
RUN cmake -S /referee/cpp -B /referee/cpp/build -DCMAKE_BUILD_TYPE=Release && cmake --build /referee/cpp/build -j8 && \
    ctest --test-dir /referee/cpp/build --output-on-failure
# the ROS 2 workspace (finds the core through ../../../../cpp)
WORKDIR /referee/ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --base-paths /referee/applications/slam/ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release \
        -DREFEREE_BUILD_TOOLS=OFF --parallel-workers 4
RUN echo "source /referee/ws/install/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/bin/bash", "-c", "source /referee/ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
