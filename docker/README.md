# Docker

Two images, both built from the repository root:

| image | Dockerfile | contents |
| --- | --- | --- |
| `referee:noetic` | `docker/ros1/Dockerfile` | ROS 1 Noetic: the C++ core + `referee_radar_slam` (Yeti odometry, ReFeree, iSAM2) + `nano_gicp` |
| `referee:humble` | `docker/ros2/Dockerfile` | ROS 2 Humble: the C++ core (tests run at build), GTSAM 4.2, `referee_odometry`, `referee_slam`, `nano_gicp`, and the Python package's tests |

```bash
docker build -f docker/ros1/Dockerfile -t referee:noetic .
docker build -f docker/ros2/Dockerfile -t referee:humble .
docker run --rm referee:humble bash /referee/docker/validate.sh          # every check
docker run --rm -v /data/MulRan/KAIST_03:/data/KAIST_03:ro -v $PWD/output:/data/output referee:humble \
    ros2 launch referee_slam referee_slam.launch.py sequence_dir:=/data/KAIST_03 save_directory:=/data/output/ dataset:=mulran
docker run --rm -v /data/MulRan/KAIST_03:/data/KAIST_03:ro -v $PWD/output:/data/output referee:noetic \
    roslaunch referee_radar_slam referee_radar_slam.launch seq_dir:=/data/KAIST_03/ save_directory:=/data/output/ dataset:=mulran
```
