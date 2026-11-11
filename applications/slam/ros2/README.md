# ReFeree radar SLAM — ROS 2 (Humble)

- `referee_odometry/`: radar odometry (ported from Yeti) that reads a sequence folder and publishes keyframes with their ReFeree descriptors (`referee_interfaces/RefereeDescriptor`).
- `referee_slam/`: pose graph (GTSAM 4.2) with ReFeree loop closing verified by nano-GICP; writes `odom_poses_kitti.txt`, `optimized_poses_kitti.txt`, `keyframe_times.txt` and the `.g2o` graph.
- `nano_gicp/`: [nano-GICP](https://github.com/engcang/nano_gicp) (MIT).

Both nodes take ReFeree from the core library ([`cpp/`](../../../cpp)). `dataset:=mulran` (0.0596 m per range bin, `referee_odometry/config/mulran.yaml`) or `oxford` (0.0432) sets the radar; a wrong value scales the trajectory. MulRan scans convert with [`../tools/mulran_to_oxford.py`](../tools/mulran_to_oxford.py).

```bash
cd ~/ros2_ws && colcon build --base-paths /path/to/ReFeree/applications/slam/ros2
ros2 launch referee_slam referee_slam.launch.py sequence_dir:=/data/KAIST_03 save_directory:=/data/output/ dataset:=mulran
```

Or with Docker: `docker build -f docker/ros2/Dockerfile -t referee:humble .` from the repository root.
