#!/usr/bin/env bash
# S4 콘 3개 배치 (s4_cones.py 산출 좌표)
for p in "6.094 0.902" "-4.418 4.428" "-6.233 -0.871"; do
  set -- $p
  ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped \
    "{header: {frame_id: 'map'}, point: {x: $1, y: $2, z: 0.0}}" >/dev/null 2>&1
  sleep 1
done
echo "콘 3개 배치 완료"
