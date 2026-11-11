# ReFeree — setup guide for coding agents

You are setting up ReFeree (radar place recognition from features and free space, IEEE RA-L 2024) for a user: the library first, then, if they want it, radar SLAM on MulRan. Run every command yourself, check its output before moving on, and stop to ask only where a step says **Ask**.

## 0. Ask the user first

**Ask**, in one message:

1. Library only (Python / C++), or SLAM too? For SLAM: **ROS 1 Noetic**, **ROS 2 Humble**, or **Docker** (recommended; no ROS on the host).
2. Where their radar data is. ReFeree reads Navtech polar scans in the Oxford layout as `<sequence>/polar/<timestamp>.png` (one row per azimuth: 8-byte timestamp, 2-byte encoder, 1-byte valid flag, then the range bins). MulRan ships `polar/` as range x azimuth without the header: convert it first (step 4).
3. Where to save results (default `~/referee_results`).

## 1. Get the code

```bash
git clone https://github.com/sparolab/ReFeree.git && cd ReFeree
```

Layout: `cpp/` C++17 core (`referee::referee`: `radar_utils` reader, `features` Cen 2018 extraction, `descriptor` range/angle descriptors, `retrieval` lookup and yaw), `python/referee` the same in Python (`pip install referee-radar`), `applications/slam/ros1` (Yeti odometry + ReFeree + iSAM2), `applications/slam/ros2` (ROS 2 port, GTSAM 4.2), `docker/ros1`, `docker/ros2`.

## 2. Library

```bash
pip install .                       # or: pip install referee-radar
cmake -S cpp -B cpp/build && cmake --build cpp/build -j && ctest --test-dir cpp/build --output-on-failure
pytest -q python/tests              # published descriptors + C++/Python parity: all must pass
```

Needs OpenCV and Eigen for the C++ (`sudo apt install libopencv-dev libeigen3-dev`; macOS `brew install opencv eigen libomp`).

## 3. SLAM

Docker (from the repository root):

```bash
docker build -f docker/ros2/Dockerfile -t referee:humble .     # or docker/ros1/Dockerfile -t referee:noetic
docker run --rm referee:humble bash /referee/docker/validate.sh # must end with: all checks passed
```

ROS 2 on the host: `colcon build --base-paths applications/slam/ros2` (GTSAM 4.2 and PCL installed). ROS 1: put `applications/slam/ros1` in a catkin workspace, `catkin build nano_gicp && catkin build`.

## 4. Run on MulRan

Convert a MulRan sequence to the Oxford layout (transpose to azimuth x range and add the 11 header bytes per row), then:

```bash
docker run --rm -v <KAIST_03>:/data/KAIST_03:ro -v ~/referee_results:/data/output referee:humble \
  ros2 launch referee_slam referee_slam.launch.py sequence_dir:=/data/KAIST_03 save_directory:=/data/output/
# ROS 1: referee:noetic roslaunch referee_radar_slam referee_radar_slam.launch seq_dir:=/data/KAIST_03/ save_directory:=/data/output/
```

Check that `optimized_poses_kitti.txt` and `odom_poses_kitti.txt` were written and that the log reports loops. On the whole of KAIST_03 the README reports 3,395 keyframes and 2,171 loops.
