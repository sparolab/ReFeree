# Multi-session SLAM

Aligns a query session to a central one, both written by the [SLAM](../slam) (ROS 1 or ROS 2), in the LT-SLAM formulation of [LT-mapper](https://github.com/gisbi-kim/lt-mapper) (Giseop Kim and Ayoung Kim, ICRA 2022): each session keeps its own frame and gets an anchor node, and inter-session loops connect the sessions through their anchors (`BetweenFactorWithAnchoring`, from LT-mapper, MIT), with a Cauchy kernel, solved with iSAM2.

- **Candidates**: every keyframe of the query session looks up the central session by its ReFeree range descriptor (cosine, as the single-session back end); the angle descriptors give the yaw between the two.
- **Verification**: the single-session loop check ([`../slam/common`](../slam/common), `registerClouds`): nano-GICP between the two keyframes' clouds seeded with that yaw, accepted when its fitness is `<= loopFitnessScoreThreshold` (200) and the translation it leaves is `<= loopMaxTranslation` (3 m).

ROS-free: file in, file out. The sessions are what the SLAM writes (`singlesession_posegraph.g2o`, `keyframe_times.txt`, `Scans/<index>,<ns>.pcd`, `RefereeDescriptors/<index>,<ns>.txt`).

```bash
# plain CMake (any machine with GTSAM, PCL, OpenCV)
cmake -S applications/multi-session-slam -B build/ms -DCMAKE_BUILD_TYPE=Release && cmake --build build/ms -j8
build/ms/multi_session_node applications/multi-session-slam/config/kaist03_split.yaml
# ROS 2 image: ros2 run referee_multi_session multi_session_node <config.yaml>
# ROS 1 image: multi_session_node <config.yaml>
```

`config/*.yaml`: `sessions_dir` holds one folder per session (`central_sess_name`, `query_sess_name`), and the result goes to `save_directory` (removed and recreated on each run): both sessions' poses in the central frame before and after the inter-session loops (`*_central_{bfr,aft}_intersession_loops.txt`, KITTI 3x4) and the accepted loops (`inter_session_loops.txt`: central node, query node).

MulRan `KAIST_03` cut in two halves (scans 1-1730 and 1731-3461), each run through the ROS 2 SLAM in its own frame (`dataset:=mulran`; single-session rigid ATE 10.8 m and 4.5 m RMSE), then `config/kaist03_split.yaml`: 1,729 candidates, **879 inter-session loops** accepted in 8 s (868 of them within 5 m in the ground truth). The distance from each keyframe of the second session to the nearest of the first goes from 30.24 m (p50) to **0.65 m**, and 77.0 % land within 5 m (13.5 % before); against the ground truth, with the first session aligned to it (rigid SE(2)), the second session's error goes from 405.2 m to **4.5 m** mean (3.9 m median). The ROS 1 image gives the same result.

## Results

`KAIST_03` cut in two halves (1730 and 1731 scans), each run through the SLAM in its own frame (single-session ATE RMSE 10.8 m and 4.5 m); 879 inter-session loops in 8 s. The distance from each keyframe of the second session to the nearest keyframe of the first (no ground truth needed), and the second session's error against ground truth once the first is aligned to it (SE(2)):

| | nearest keyframe, p50 | within 5 m | error vs ground truth, mean (median) |
| --- | --- | --- | --- |
| before | 30.24 m | 13.5 % | 405.2 m (457.0 m) |
| after | **0.65 m** | **77.0 %** | **4.5 m (3.9 m)** |

Only part of the second half retraces the first, so not every keyframe has a counterpart to land on.
