"""The README GIF: a ReFeree SLAM session replayed keyframe by keyframe.

    python tools/gif_features.py <session> --polar <polar dir> --out <session>/gif_features.npz
    python tools/make_gif.py <session> --polar <polar dir> --gt xypose.csv --out assets/referee_slam.gif

<session>: poses.txt, odom_poses.txt, times.txt, loops.csv, gif_features.npz (per-keyframe radar features and
ReFeree range descriptors). Left: the radar scan and its descriptor; right: the radar feature map on the
optimized poses, the odometry, the SLAM trajectory and its loops.
"""
import argparse
import io
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter, map_coordinates

BG, FG, MUTED = "#0d1117", "#e6edf3", "#7d8590"
BLUE, GREEN, GREY, ORANGE = "#3b82f6", "#22c55e", "#6b7280", "#f59e0b"


def read_pcd(path):
    raw = Path(path).read_bytes()
    i = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
    return np.frombuffer(raw[i:], np.float32).reshape(-1, 4)


def cart(polar_png, resolution, header=11, radius=80.0, size=360, reverse=False):
    im = np.asarray(Image.open(polar_png), dtype=float)[:, header:]   # (azimuths, range bins)
    na, nr = im.shape
    yy, xx = np.mgrid[0:size, 0:size]
    x = (size / 2 - yy) / (size / 2) * radius                         # up = forward
    y = (size / 2 - xx) / (size / 2) * radius                         # left
    rho = np.hypot(x, y) / resolution
    az = (np.arctan2(y, x) % (2 * np.pi)) / (2 * np.pi) * na
    if reverse:
        az = (na - az) % na
    out = map_coordinates(im, [az, rho], order=1, cval=0, mode="grid-wrap")
    out[np.hypot(x, y) > radius] = np.nan
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("run")
    p.add_argument("--polar", required=True)
    p.add_argument("--gt", help="MulRan xypose.csv (t [ns], x, y, ...)")
    p.add_argument("--resolution", type=float, default=0.0596)
    p.add_argument("--reverse-azimuth", action="store_true")
    p.add_argument("--name", default="ReFeree")
    p.add_argument("--title", default="MulRan KAIST_03")
    p.add_argument("--every", type=int, default=20)
    p.add_argument("--fps", type=int, default=12)
    p.add_argument("--grid", type=float, default=1.0, help="map cell (m)")
    p.add_argument("--out", required=True)
    a = p.parse_args()

    r = Path(a.run)
    P = np.loadtxt(r / "poses.txt")
    O = np.loadtxt(r / "odom_poses.txt")
    n = min(len(P), len(O))
    xy, yaw = P[:n][:, [3, 7]], np.arctan2(P[:n, 4], P[:n, 0])
    oxy = O[:n][:, [3, 7]]
    t = np.loadtxt(r / "times.txt")[:n]
    F = np.load(r / "gif_features.npz")
    scans = list(F["scans"])[:n]
    D = F["descriptors"][:n]
    starts = np.r_[0, np.cumsum(F["counts"])]
    feats = F["points"]
    loops = [tuple(map(int, l.split(",")[:2])) for l in (r / "loops.csv").read_text().splitlines()[1:] if l.strip()]
    loops = [(max(i, j), min(i, j)) for i, j in loops if max(i, j) < n]

    gt = None
    if a.gt:
        g = np.loadtxt(a.gt, delimiter=",")
        Q = np.c_[np.interp(t * 1e9, g[:, 0], g[:, 1]), np.interp(t * 1e9, g[:, 0], g[:, 2])]
        A, B = Q - Q.mean(0), xy - xy.mean(0)
        U, _, Vt = np.linalg.svd(A.T @ B)
        R = Vt.T @ np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
        gt = A @ R.T + xy.mean(0)

    allp = np.concatenate([xy, oxy] + ([gt] if gt is not None else []))
    span = np.ptp(xy, 0)
    lo, hi = xy.min(0) - 0.08 * span.max(), xy.max(0) + 0.08 * span.max()
    gx, gy = np.arange(lo[0], hi[0], a.grid), np.arange(lo[1], hi[1], a.grid)
    count = np.zeros((len(gy), len(gx)))

    def paint(k):
        q = feats[starts[k]:starts[k + 1]]
        c, s = np.cos(yaw[k]), np.sin(yaw[k])
        w = q @ np.array([[c, s], [-s, c]]) + xy[k]
        ix, iy = ((w[:, 0] - lo[0]) / a.grid).astype(int), ((w[:, 1] - lo[1]) / a.grid).astype(int)
        ok = (ix >= 0) & (ix < len(gx)) & (iy >= 0) & (iy < len(gy))
        np.add.at(count, (iy[ok], ix[ok]), 1)

    loop_at = {}
    for q, m in loops:
        loop_at.setdefault(q, []).append(m)
    ks = list(range(0, n, a.every)) + [n - 1] * a.fps * 2
    frames, painted, last_loop = [], -1, None
    plt.rcParams.update({"font.family": "DejaVu Sans", "text.color": FG})
    for k in ks:
        while painted < k:
            painted += 1
            paint(painted)
        fig = plt.figure(figsize=(12.8, 6.4), dpi=80, facecolor=BG)
        ax_img = fig.add_axes([0.03, 0.33, 0.33, 0.58])
        ax_d = fig.add_axes([0.05, 0.08, 0.30, 0.18])
        ax_map = fig.add_axes([0.40, 0.03, 0.58, 0.90])
        ax_img.imshow(cart(Path(a.polar) / scans[k], a.resolution, reverse=a.reverse_azimuth), cmap="gray", vmin=0,
                      vmax=120)
        ax_img.axis("off")
        ax_img.set_title(f"radar scan  ·  keyframe {k}", color=FG, fontsize=13, loc="left")
        recent = [q for q in loop_at if k - a.every < q <= k]
        if recent:
            last_loop = (recent[-1], loop_at[recent[-1]][-1])
        ax_d.set_facecolor(BG)
        if recent:
            q, m = last_loop
            ax_d.plot(D[q], "-", c=ORANGE, lw=2, label=f"keyframe {q}")
            ax_d.plot(D[m], "--", c=GREEN, lw=2, label=f"match {m}")
            ax_img.text(0.98, 0.02, f"LOOP  {q} → {m}", transform=ax_img.transAxes, ha="right", va="bottom",
                        color=BG, fontsize=13, weight="bold", bbox=dict(boxstyle="round,pad=0.35", fc=GREEN, ec="none"))
        else:
            ax_d.plot(D[k], "-", c=ORANGE, lw=2, label=f"keyframe {k}")
        ax_d.set_xticks([]); ax_d.set_yticks([])
        for sp in ax_d.spines.values():
            sp.set_color(MUTED)
        ax_d.set_title(f"{a.name} descriptor: free space in {D.shape[1]} range bands", color=MUTED, fontsize=11, loc="left")
        ax_d.legend(loc="upper right", facecolor=BG, edgecolor=MUTED, labelcolor=FG, fontsize=8)

        ax_map.set_facecolor(BG)
        img = np.log1p(gaussian_filter(count, 0.6))
        ax_map.imshow(img / max(img.max(), 1e-6), cmap="bone", origin="lower", vmin=0, vmax=0.8,
                      extent=[lo[0], lo[0] + len(gx) * a.grid, lo[1], lo[1] + len(gy) * a.grid])
        if gt is not None:
            ax_map.plot(gt[: k + 1, 0], gt[: k + 1, 1], ":", c=FG, lw=1.0, alpha=0.6, label="ground truth")
        ax_map.plot(oxy[: k + 1, 0], oxy[: k + 1, 1], "--", c=GREY, lw=1.2, label="radar odometry")
        segs = [(xy[q], xy[m]) for q, m in loops if q <= k]
        if segs:
            from matplotlib.collections import LineCollection
            ax_map.add_collection(LineCollection(segs, colors=GREEN, linewidths=0.6, alpha=0.6))
        ax_map.plot(xy[: k + 1, 0], xy[: k + 1, 1], "-", c=BLUE, lw=1.8, label=f"+ {a.name} loops")
        ax_map.plot(*xy[k], "o", c=ORANGE, ms=7)
        ax_map.set_xlim(lo[0], hi[0]); ax_map.set_ylim(lo[1], hi[1]); ax_map.set_aspect("equal")
        ax_map.axis("off")
        ax_map.set_title(f"{a.title}  ·  {sum(1 for q, _ in loops if q <= k)} loops", color=FG, fontsize=13, loc="left")
        ax_map.legend(loc="lower right", facecolor=BG, edgecolor=MUTED, labelcolor=FG, fontsize=10)
        buf = io.BytesIO()
        fig.savefig(buf, format="png", facecolor=BG)
        plt.close(fig)
        frames.append(Image.open(buf).convert("RGB"))
    pal = frames[-1].quantize(colors=128, method=Image.Quantize.MEDIANCUT)
    qf = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    qf[0].save(a.out, save_all=True, append_images=qf[1:], duration=int(1000 / a.fps), loop=0, optimize=True)
    print(f"{a.out}: {len(frames)} frames, {Path(a.out).stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
