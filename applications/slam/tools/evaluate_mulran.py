"""Turn a raw ReFeree SLAM output folder into a session and score it against MulRan ground truth.

    REFEREE_SCANS=<converted>/KAIST_03/polar MULRAN_GT=<MulRan>/KAIST_03/xypose.csv \
        python3 evaluate_mulran.py <save_directory> <session_dir> <ros1|ros2>

Session: poses.txt (optimized, KITTI 3x4), odom_poses.txt, times.txt (s), loops.csv (previous,current keyframe),
keyframe_scan_index.txt (index into the sorted polar/ scans), gt_xy.txt (MulRan GT at the keyframes, x y).
ATE: rigid SE(2) fit (rotation + translation, no scale) of the keyframe positions onto the GT; the similarity fit
(with scale) is printed too, labelled.
"""
import os, shutil, sys
import numpy as np

raw, out, kind = sys.argv[1], sys.argv[2], sys.argv[3]
SCANS = os.environ.get('REFEREE_SCANS', 'KAIST_03/polar')  # the converted polar/ folder
GT = os.environ.get('MULRAN_GT', 'KAIST_03/xypose.csv')  # MulRan ground truth: t [ns], x, y, ...
os.makedirs(out, exist_ok=True)
if kind == 'ros2':
    P, O, T = 'optimized_poses_kitti.txt', 'odom_poses_kitti.txt', 'keyframe_times.txt'
else:
    P, O, T = 'optimized_poses.txt', 'odom_poses.txt', 'times.txt'
opt = np.loadtxt(os.path.join(raw, P), ndmin=2)
odo = np.loadtxt(os.path.join(raw, O), ndmin=2)
t = np.loadtxt(os.path.join(raw, T), ndmin=1)
n = min(len(opt), len(odo), len(t))
opt, odo, t = opt[:n], odo[:n], t[:n]
# loops: non-consecutive edges of the g2o graph (ROS 1 also writes loops.csv)
loops = []
if os.path.exists(os.path.join(raw, 'loops.csv')):  # ROS 1 writes the loops it added
    for line in open(os.path.join(raw, 'loops.csv')).read().split()[1:]:
        i, j = map(int, line.split(','))
        if max(i, j) < n:
            loops.append((min(i, j), max(i, j)))
for line in open(os.path.join(raw, 'singlesession_posegraph.g2o')):
    p = line.split()
    if p and p[0].startswith('EDGE'):
        i, j = int(p[1]), int(p[2])
        if abs(i - j) > 1 and max(i, j) < n:
            loops.append((min(i, j), max(i, j)))
stamps = np.array(sorted(int(f[:-4]) for f in os.listdir(SCANS) if f.endswith('.png')))
idx = np.searchsorted(stamps, np.round(t * 1e9).astype(np.int64))
idx = np.clip(idx, 0, len(stamps) - 1)
idx = np.where(np.abs(stamps[np.clip(idx - 1, 0, None)] - t * 1e9) < np.abs(stamps[idx] - t * 1e9), idx - 1, idx)
g = np.loadtxt(GT, delimiter=',')
G = np.c_[np.interp(t * 1e9, g[:, 0], g[:, 1]), np.interp(t * 1e9, g[:, 0], g[:, 2])]
np.savetxt(os.path.join(out, 'poses.txt'), opt, fmt='%.6f')
np.savetxt(os.path.join(out, 'odom_poses.txt'), odo, fmt='%.6f')
np.savetxt(os.path.join(out, 'times.txt'), t, fmt='%.9f')
np.savetxt(os.path.join(out, 'keyframe_scan_index.txt'), idx, fmt='%d')
np.savetxt(os.path.join(out, 'gt_xy.txt'), G, fmt='%.4f')
with open(os.path.join(out, 'loops.csv'), 'w') as f:
    f.write('previous,current\n')
    for i, j in sorted(set(loops)):
        f.write(f'{i},{j}\n')
for name in ('singlesession_posegraph.g2o', 'run.log'):
    if os.path.exists(os.path.join(raw, name)):
        shutil.copy(os.path.join(raw, name), out)


def fit(Q, G, scale):
    mq, mg = Q.mean(0), G.mean(0)
    A, B = Q - mq, G - mg
    U, S, Vt = np.linalg.svd(A.T @ B)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, d])
    R = Vt.T @ D @ U.T
    s = (S * np.diag(D)).sum() / (A ** 2).sum() if scale else 1.0
    e = np.linalg.norm(s * A @ R.T + mg - G, axis=1)
    return e, s


print(f'{kind}: {n} keyframes, {len(set(loops))} loops in the graph, scans {idx.min()}..{idx.max()}')
for label, X in (('odometry', odo), ('SLAM', opt)):
    Q = X[:, [3, 7]]
    e, _ = fit(Q, G, False)
    es, s = fit(Q, G, True)
    print(f'  {label:9s} rigid ATE (no scale): RMSE {np.sqrt((e**2).mean()):.2f} m, mean {e.mean():.2f} m, '
          f'max {e.max():.2f} m | with scale (s={s:.3f}): RMSE {np.sqrt((es**2).mean()):.2f} m')
    print(f'  {label:9s} final position error after the rigid fit: {e[-1]:.2f} m')
