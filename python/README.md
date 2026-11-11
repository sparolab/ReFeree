# referee (Python) — `pip install referee-radar`

Mirrors the C++ core under `cpp/` module for module, and is checked
against the descriptors the published implementation produced. The C++ side is
what the ROS 2 nodes use; this side is for reproducing the paper and for trying
things out.

```
referee/
  radar_utils.py   scan loading, detections back to a polar image
  features.py      feature extraction after Cen and Newman, ICRA 2018
  descriptor.py    range (rotation invariant) and angle (yaw) descriptors
  retrieval.py     place lookup and yaw recovery
scripts/demo.py    match a query against candidates, recover the yaw
tests/             equivalence with the published C++
```

## Install

```bash
pip install referee-radar        # or, from the repository root: pip install ".[test]"
python3 -m pytest python/tests -v
```

`tests/test_cpp.py` also checks this package against the C++ core when `cpp/build/referee_describe` is built.

## Use

```python
import referee

scan = referee.load_polar_scan("000000.png", referee.NavtechModel.CIR204)
polar = referee.targets_to_polar_image(scan.power.shape,
                                       referee.cen2018_features(scan.power))

db, yaw_db = referee.RangeDescriptorDatabase(), referee.AngleDescriptorDatabase()
db.add(referee.make_range_descriptor(polar))
yaw_db.add(referee.make_angle_descriptor(polar))

match = db.detect_loop_closure(query_range)
if match:
    yaw = yaw_db.yaw_difference(query_angle, match.index)
```

## Rewritten from the released version

The released Python was not a port of the released C++ — the two computed
different descriptors. The C++ is what the RA-L results came from, so it is
canonical and this package now reproduces it: the range descriptor to the last
bit, the angular one to 5e-10, which is float accumulation order.

What changed, and why:

* **Feature extraction now matches the C++.** The released Python replaced the
  median filter and Gaussian smoothing with a mean and a uniform filter; both
  originals were left in the file, commented out.
* **The range descriptor now uses the C++ window alignment.** The released
  Python windowed correctly, which the C++ does not — see below.
* **The angular descriptor now counts what the C++ counts**: bins from the
  outermost detection inwards, over 50 sectors. The released Python counted
  only the empty bins below that detection, over 40.
* **Yaw estimation works.** The released demo looped over `shift_index` without
  ever using it, so every iteration scored the same unshifted pair and the
  reported heading was always zero. It also took `argmin` of a similarity.

## Two errors kept on purpose

Both are in the published C++, so both are on by default and fixing either
changes published numbers.

| flag | what the published code does |
| --- | --- |
| `RangeDescriptorParams.legacy_window_alignment` | resets the accumulator at the *start* of each window, so band 0 holds one range bin rather than 80, every other band is shifted by one, and the trailing 79 bins are dropped |
| `AngleDescriptorParams.legacy_skip_nearest_bin` | scans each ray down to bin 1, never examining bin 0 |

```python
fixed = referee.make_range_descriptor(
    polar, referee.RangeDescriptorParams(legacy_window_alignment=False))
```
