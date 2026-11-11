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

Layout: `cpp/` C++17 core (`referee::referee`: `radar_utils` reader, `features` Cen 2018 extraction, `descriptor` range/angle descriptors, `retrieval` lookup and yaw), `python/referee` the same in Python (`pip install referee-radar`), `applications/slam/common` (ROS-free odometry and back end), `applications/slam/ros1` and `applications/slam/ros2` (the same two nodes on each ROS), `applications/slam/tools` (MulRan converter, evaluation), `applications/multi-session-slam` (LT-SLAM style alignment of two sessions, ROS-free), `docker/ros1`, `docker/ros2`.

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

ROS 2 on the host: `colcon build --base-paths applications/slam/ros2` (GTSAM 4.2 and PCL installed). ROS 1: `catkin config --source-space applications/slam/ros1 && catkin build` (GTSAM, PCL).

## 4. Run on MulRan

Convert a MulRan sequence to the Oxford layout (azimuth x range, azimuth order reversed to the Oxford encoder direction, 11 header bytes per row) — without the reversal the trajectory comes out mirrored:

```bash
python3 applications/slam/tools/mulran_to_oxford.py <MulRan>/KAIST_03/polar <converted>/KAIST_03/polar
```

Then (`dataset:=mulran` sets MulRan's 0.0596 m per range bin; `oxford` is 0.0432):

```bash
docker run --rm -v <KAIST_03>:/data/KAIST_03:ro -v ~/referee_results:/data/output referee:humble \
  ros2 launch referee_slam referee_slam.launch.py sequence_dir:=/data/KAIST_03 save_directory:=/data/output/ dataset:=mulran
# ROS 1: referee:noetic roslaunch referee_radar_slam referee_radar_slam.launch seq_dir:=/data/KAIST_03/ save_directory:=/data/output/ dataset:=mulran
```

Check that `optimized_poses_kitti.txt`, `odom_poses_kitti.txt` and `keyframe_times.txt` were written (both ROS versions) and that the log reports loops. Score it against the ground truth:

```bash
REFEREE_SCANS=<converted>/KAIST_03/polar MULRAN_GT=<MulRan>/KAIST_03/xypose.csv \
  python3 applications/slam/tools/evaluate_mulran.py ~/referee_results ~/referee_results/session ros2
```

On the whole of KAIST_03 expect 3,459 keyframes, about 1,300 loop edges and a rigid (no-scale) ATE RMSE near 6.2 m for SLAM against 80.4 m for the odometry alone, on ROS 2 and ROS 1 alike (they share `applications/slam/common`); the odometry log warns that 4 scans kept the previous motion.

## 5. Multi-session (optional)

Two sessions of the same place, each written by the SLAM in its own frame (both ROS versions write `Scans/` and `RefereeDescriptors/` next to the graph). For a test, split a converted MulRan sequence in two folders (KAIST_03: the first 1730 scans and the remaining 1731, as `KAIST_03_01/polar` and `KAIST_03_02/polar`) and run the SLAM on each with `save_directory:=<sessions>/01/` and `<sessions>/02/`. Then:

```bash
docker run --rm -v <sessions>:/data/output/sessions referee:humble \
  ros2 run referee_multi_session multi_session_node /referee/applications/multi-session-slam/config/kaist03_split.yaml
# ROS 1 image: referee:noetic multi_session_node <config.yaml>; without Docker: see applications/multi-session-slam/README.md
```

It writes `<sessions>/LT_result/`: both sessions in the central frame before and after (`*_central_{bfr,aft}_intersession_loops.txt`) and `inter_session_loops.txt`. On KAIST_03 split in two expect about 880 inter-session loops, and the second session's keyframes within 0.7 m (p50) of the first's after alignment (30 m before), 77 % within 5 m.
