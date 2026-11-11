# ReFeree radar SLAM — ROS 1 (Noetic)

The former [ReFeree-Radar-SLAM](https://github.com/sparolab/ReFeree-Radar-SLAM), merged here:

- `referee_radar_slam/`: [Yeti radar odometry](https://github.com/keenan-burnett/yeti_radar_odometry) (Keenan Burnett, BSD-3-Clause) publishes keyframe clouds and the ReFeree descriptors of each scan (`referee_odometry`); the pose graph (`referee_posegraph`) finds loops with ReFeree, verifies them with nano-GICP and optimizes with iSAM2. The structure follows [Navtech-Radar-SLAM](https://github.com/gisbi-kim/navtech-radar-slam).
- `nano_gicp/`: [nano-GICP](https://github.com/engcang/nano_gicp) (MIT).

ReFeree itself is no longer a private copy: both nodes use the repository's core library ([`cpp/`](../../../cpp)), which reproduces the published descriptors and lookup (loop threshold 0.9 cosine, 30 recent keyframes excluded, as before).

```bash
cd ~/catkin_ws/src && ln -s /path/to/ReFeree/applications/slam/ros1 referee
cd .. && catkin build nano_gicp && catkin build
roslaunch referee_radar_slam referee_radar_slam.launch seq_dir:=/data/KAIST_03/ save_directory:=/data/output/ dataset:=mulran rviz:=true
```

`seq_dir` holds `polar/<timestamp>.png` in the Oxford layout (MulRan: convert with [`../tools/mulran_to_oxford.py`](../tools/mulran_to_oxford.py)). `dataset:=mulran` (0.0596 m per range bin) or `oxford` (0.0432) sets the radar; a wrong value scales the trajectory. The session written to `save_directory`: `odom_poses.txt`, `optimized_poses.txt` (KITTI), `times.txt` (each keyframe's scan time, s), `loops.csv` and the `.g2o` graph. Or with Docker: `docker build -f docker/ros1/Dockerfile -t referee:noetic .` from the repository root.
