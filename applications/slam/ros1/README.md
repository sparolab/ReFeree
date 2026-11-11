# ReFeree radar SLAM — ROS 1 (Noetic)

`referee_radar_slam` replaces [ReFeree-Radar-SLAM](https://github.com/sparolab/ReFeree-Radar-SLAM). It runs the same two nodes as the ROS 2 packages, built from the same ROS-free code in [`../common`](../common):

- `odometry_node`: radar odometry after [Yeti](https://github.com/keenan-burnett/yeti_radar_odometry) (Keenan Burnett, BSD-3-Clause): Cen 2018 features, ORB matching, motion-distorted RANSAC with Doppler correction (adaptive, with a constant-velocity fallback when a match fails). It publishes `odom`, `cloud_local` and the ReFeree descriptors of each scan (`referee`).
- `pose_graph_node`: keyframes, ReFeree loop search (candidates at least 10 m back along the path), verification by [nano-GICP](https://github.com/engcang/nano_gicp) seeded with the descriptor's yaw (fitness <= 200, residual translation <= 3 m), Cauchy-robust loop factors, iSAM2.

```bash
cd ~/catkin_ws && catkin config --source-space /path/to/ReFeree/applications/slam/ros1 && catkin build
roslaunch referee_radar_slam referee_radar_slam.launch seq_dir:=/data/KAIST_03/ save_directory:=/data/output/ dataset:=mulran rviz:=true
```

`seq_dir` holds `polar/<timestamp>.png` in the Oxford layout (MulRan: convert with [`../tools/mulran_to_oxford.py`](../tools/mulran_to_oxford.py)). `dataset:=mulran` (0.0596 m per range bin) or `oxford` (0.0432) sets the radar; a wrong value scales the trajectory. Written to `save_directory`, as on ROS 2: `odom_poses_kitti.txt`, `optimized_poses_kitti.txt`, `keyframe_times.txt` (each keyframe's scan time, s) and `singlesession_posegraph.g2o`. Or with Docker: `docker build -f docker/ros1/Dockerfile -t referee:noetic .` from the repository root.
