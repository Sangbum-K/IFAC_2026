#!/usr/bin/env bash
# 브리지 open-loop 재생. 사용: replay_bridge.sh <true|false> <출력이름>
#   $1 = bridge/reassoc_enabled
set +u
FLAG="$1"; TAG="$2"; RATE="${3:-1.0}"
BAG=/home/sangbum/IFAC_ws/rosbag/run_0814_191443
SHARE=/home/sangbum/IFAC_ws/planning_ws/install/planning_pkg/share/planning_pkg/config
DIR=/home/sangbum/IFAC_ws/tools/bag_analysis_0814
OUT=$DIR/bridge_${TAG}

source /opt/ros/humble/setup.bash
source /home/sangbum/IFAC_ws/planning_ws/install/setup.bash

pkill -9 -x perception_obstacle_bridge_node.py; sleep 1
rm -rf "$OUT"
ros2 run planning_pkg perception_obstacle_bridge_node.py --ros-args \
  --params-file "$SHARE/params.yaml" \
  --params-file "$SHARE/environments/real_car.yaml" \
  -p use_sim_time:=true \
  -p bridge/reassoc_enabled:="$FLAG" > "$DIR/bridge_${TAG}.log" 2>&1 &
sleep 3
ros2 bag record -o "$OUT" /perception/obstacles_frenet > /dev/null 2>&1 &
REC=$!
sleep 2
ros2 bag play "$BAG" --clock 200 --rate "$RATE" --topics \
  /perception/obstacles /car_state/frenet/odom /global_waypoints \
  /tf /tf_static /map > /dev/null 2>&1
sleep 2
kill -INT "$REC" 2>/dev/null; sleep 2
pkill -9 -x perception_obstacle_bridge_node.py
echo "done: $OUT (재연결 로그 $(grep -c '트랙 재연결' "$DIR/bridge_${TAG}.log"), 흡수 로그 $(grep -c '중복 검출 흡수' "$DIR/bridge_${TAG}.log"))"
