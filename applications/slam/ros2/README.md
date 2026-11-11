# ReFeree radar SLAM — ROS 2 (Humble)

- `referee_odometry/`: radar odometry (after Yeti) that reads a sequence folder and publishes `odom`, `cloud_local` and the ReFeree descriptors of each scan (`referee_interfaces/RefereeDescriptor`).
- `referee_slam/`: the pose graph (GTSAM): ReFeree loop search, nano-GICP verification (fitness <= 200, residual translation <= 3 m), Cauchy-robust loop factors, iSAM2. Writes `odom_poses_kitti.txt`, `optimized_poses_kitti.txt`, `keyframe_times.txt` and the `.g2o` graph.
- `referee_interfaces/`: the descriptor message.

The estimation is the ROS-free code in [`../common`](../common), shared with the ROS 1 package, and ReFeree comes from the core library ([`cpp/`](../../../cpp)).

```bash
cd ~/ros2_ws && colcon build --base-paths /path/to/ReFeree/applications/slam/ros2
ros2 launch referee_slam referee_slam.launch.py sequence_dir:=/data/KAIST_03 save_directory:=/data/output/ dataset:=mulran
```

`dataset:=mulran` (0.0596 m per range bin, `referee_odometry/config/mulran.yaml`) or `oxford` (0.0432) sets the radar; a wrong value scales the trajectory. MulRan scans convert with [`../tools/mulran_to_oxford.py`](../tools/mulran_to_oxford.py). Or with Docker: `docker build -f docker/ros2/Dockerfile -t referee:humble .` from the repository root.
