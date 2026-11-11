# Multi-robot SLAM with Commerge

[Commerge](https://github.com/sparolab/Commerge) (IJRR 2026) merges the maps of two robots that never met over a weak link: from the affinity of their place descriptors it selects the few keyframes worth exchanging (sequential matching, balanced vertex cover, consistent clique), registers them and merges the maps. It never looks at the sensor, so a radar descriptor works as well as a LiDAR one.

Its [radar demo](https://github.com/sparolab/Commerge/tree/main/tools/demo/radar) splits MulRan into two radar robots, one moved into its own frame, and merges them from radar band descriptors in the spirit of ReFeree's range descriptor:

```bash
pip install commerge referee-radar
# one session per robot (keyframe poses + descriptors), the (N_A, N_B) affinity of the range descriptors, then:
commerge robot_a robot_b --affinity affinity.npy --threshold 0.9 --registration pairs --policy all --out merged/
```
