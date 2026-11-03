#!/bin/sh
# Build geo_intersect against the vendored geogram (thirdparty/geogram, see VENDORED.md).
# usage: build.sh <build dir>
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
G="$HERE/../../thirdparty/geogram"
B="${1:?build dir}"
mkdir -p "$B/geogram"
cmake -S "$G" -B "$B/geogram" -G Ninja -DCMAKE_BUILD_TYPE=Release -DGEOGRAM_LIB_ONLY=ON -DGEOGRAM_WITH_GRAPHICS=OFF \
  -DGEOGRAM_WITH_LUA=OFF -DGEOGRAM_WITH_HLBFGS=OFF -DGEOGRAM_WITH_TETGEN=OFF -DGEOGRAM_WITH_TRIANGLE=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
ninja -C "$B/geogram" geogram
LIB="$(find "$B/geogram" -name 'libgeogram.*' | head -1)"
INC="$(find "$B/geogram" -type d -name geogram_include 2>/dev/null | head -1)"
c++ -O2 -std=c++17 -I"$G/src/lib" ${INC:+-I"$INC"} -I"$B/geogram" "$HERE/geo_intersect.cpp" "$LIB" \
  -Wl,-rpath,"$(dirname "$LIB")" -o "$B/geo_intersect"
echo "built $B/geo_intersect"
