# Multi-session SLAM (planned)

Aligning several SLAM sessions of the same place (different days, weather, directions) through ReFeree loops between them: the range descriptor finds the place in another session, the angle descriptor gives the yaw, ICP refines it, and one pose graph joins the sessions.

Not released yet. Until then, [RadLoc](https://github.com/sparolab/RadLoc) (radar global localization, same lab) aligns radar sessions, and [Commerge](https://github.com/sparolab/Commerge) merges two robots' maps from any place descriptor ([multi-robot-slam](../multi-robot-slam)).
