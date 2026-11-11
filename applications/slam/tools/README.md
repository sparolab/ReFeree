# Tools

- `mulran_to_oxford.py`: MulRan `polar/` scans (range x azimuth PNG) to the Oxford layout both SLAMs read (azimuth x range, azimuth order reversed to the Oxford encoder direction, 11 header bytes per row with a per-azimuth time). Without the reversal the trajectory comes out mirrored.
- `evaluate_mulran.py`: turns a SLAM output folder into a session (`poses.txt`, `odom_poses.txt`, `times.txt`, `loops.csv`, `keyframe_scan_index.txt`, `gt_xy.txt`) and prints the ATE against the MulRan ground truth (`xypose.csv`) with a rigid SE(2) fit, no scale (a scale fit is printed too, labelled).

```bash
python3 mulran_to_oxford.py <MulRan>/KAIST_03/polar <converted>/KAIST_03/polar
REFEREE_SCANS=<converted>/KAIST_03/polar MULRAN_GT=<MulRan>/KAIST_03/xypose.csv \
    python3 evaluate_mulran.py <save_directory> <session_dir> ros2     # or ros1
```
