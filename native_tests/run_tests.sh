#!/bin/bash
# Host-side (PC) tests for the native safety layer and object tracker.
# Synthetic ray-cast depth scenes / simulated people; no phone needed.
set -e
cd "$(dirname "$0")"
SRC=../android/app/src/main/cpp
OUT=${TMPDIR:-/tmp}/hsv2_native_tests
mkdir -p "$OUT"
for t in test_safety test_walk test_walkstep test_semantic test_fixes; do
  g++ -std=c++17 -O2 -Wall -I"$SRC" "$t.cpp" "$SRC/safety.cpp" -o "$OUT/$t"
  echo "== $t"; "$OUT/$t"
done
g++ -std=c++17 -O2 -Wall -I"$SRC" test_gravity.cpp "$SRC/safety.cpp" "$SRC/gravity_calib.cpp" "$SRC/rigid.cpp" -o "$OUT/test_gravity"
echo "== test_gravity"; "$OUT/test_gravity"
g++ -std=c++17 -O2 -Wall -I"$SRC" test_vo.cpp "$SRC/vo.cpp" "$SRC/recorder.cpp" "$SRC/rigid.cpp" -lpthread -o "$OUT/test_vo"
echo "== test_vo"; "$OUT/test_vo"
g++ -std=c++17 -O2 -Wall -I"$SRC" test_map.cpp "$SRC/local_map.cpp" "$SRC/safety.cpp" -o "$OUT/test_map"
echo "== test_map"; "$OUT/test_map"
g++ -std=c++17 -O2 -Wall -I"$SRC" test_plan.cpp "$SRC/local_map.cpp" "$SRC/safety.cpp" "$SRC/nav_plan.cpp" "$SRC/view3d.cpp" -o "$OUT/test_plan"
echo "== test_plan"; "$OUT/test_plan"
g++ -std=c++17 -O2 -Wall -I"$SRC" test_trip.cpp "$SRC/local_map.cpp" "$SRC/safety.cpp" -o "$OUT/test_trip"
echo "== test_trip"; "$OUT/test_trip"
g++ -std=c++17 -O2 -Wall -I"$SRC" test_recorder.cpp "$SRC/recorder.cpp" "$SRC/vo.cpp" "$SRC/rigid.cpp" -lpthread -o "$OUT/test_recorder"
echo "== test_recorder"; "$OUT/test_recorder"
g++ -std=c++17 -O2 -Wall -I"$SRC" test_guidance.cpp "$SRC/guidance.cpp" "$SRC/safety_validator.cpp" "$SRC/local_map.cpp" "$SRC/safety.cpp" -o "$OUT/test_guidance"
echo "== test_guidance"; "$OUT/test_guidance"
g++ -std=c++17 -O2 -Wall -I"$SRC" test_health.cpp "$SRC/sensor_health.cpp" -o "$OUT/test_health"
echo "== test_health"; "$OUT/test_health"
g++ -std=c++17 -O2 -Wall -I"$SRC" test_validator.cpp "$SRC/guidance.cpp" "$SRC/safety_validator.cpp" "$SRC/local_map.cpp" "$SRC/safety.cpp" -o "$OUT/test_validator"
echo "== test_validator"; "$OUT/test_validator"
# The tracker includes QNN headers (via yolo_detector.h).
QNN=${QNN_SDK_ROOT:-/home/naveen/Downloads/qairt/2.47.0.260601}/include
g++ -std=c++17 -O2 -Wall -I"$SRC" -I"$QNN" -I"$QNN/QNN" -I"$QNN/QNN/System" test_tracker.cpp "$SRC/tracker.cpp" -o "$OUT/test_tracker"
echo "== test_tracker"; "$OUT/test_tracker"
