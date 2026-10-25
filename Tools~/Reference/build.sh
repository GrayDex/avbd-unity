#!/bin/sh
# Builds the reference dump against a clone of avbd-demo3d and writes Tests/Editor/Reference.bytes.
# Usage (WSL): ./build.sh /mnt/c/Users/serge/dev/dev.references/avbd-demo3d
set -e
demo="$1/source"
here="$(cd "$(dirname "$0")" && pwd)"
out="$(mktemp -d)"
g++ -O2 -std=c++17 -I"$here/stub" -I"$demo" "$here/reference.cpp" \
    "$demo/solver.cpp" "$demo/force.cpp" "$demo/rigid.cpp" "$demo/joint.cpp" "$demo/spring.cpp" \
    "$demo/manifold.cpp" "$demo/collide.cpp" -o "$out/reference"
"$out/reference" "$here/../../Tests/Editor/Reference.bytes"
rm -r "$out"
