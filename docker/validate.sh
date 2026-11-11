#!/bin/bash
# Runs every check the repository ships (inside the ROS 2 image: docker run --rm referee:humble bash /referee/docker/validate.sh).
set -uo pipefail
ROOT=${REFEREE_ROOT:-/referee}
failures=0
step() { printf '\n\033[1m== %s\033[0m\n' "$1"; }

step "C++ core: published descriptors, retrieval and yaw"
ctest --test-dir "$ROOT/cpp/build" --output-on-failure || failures=$((failures+1))

step "Python package: published descriptors and C++ parity"
cd "$ROOT" && python3 -m pytest python/tests -q || failures=$((failures+1))

printf '\n'
if [ "$failures" -eq 0 ]; then echo "all checks passed"; else echo "$failures check(s) failed"; fi
exit $((failures == 0 ? 0 : 1))
