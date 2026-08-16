#!/usr/bin/env bash
# spliner_node open-loop 재생 (planning-stop-latch-rosbag0806 메모의 방법).
# 사용: replay_ab.sh <출력이름> [rate]
set +u
TAG="$1"; RATE="${2:-2.0}"
BAG=/home/sangbum/IFAC_ws/rosbag/run_0814_191443
SHARE=/home/sangbum/IFAC_ws/planning_ws/install/planning_pkg/share/planning_pkg/config
OUT=/home/sangbum/IFAC_ws/tools/bag_analysis_0814/replay_${TAG}.log

source /opt/ros/humble/setup.bash
source /home/sangbum/IFAC_ws/planning_ws/install/setup.bash

pkill -9 -x spliner_node; sleep 1
ros2 run planning_pkg spliner_node --ros-args \
  --params-file "$SHARE/params.yaml" \
  --params-file "$SHARE/environments/real_car.yaml" \
  -p use_sim_time:=true > "$OUT" 2>&1 &
NODE=$!
sleep 3
ros2 bag play "$BAG" --clock 200 --rate "$RATE" --topics \
  /perception/obstacles_frenet /car_state/frenet/odom \
  /global_waypoints /opponent_prediction/obstacles > /dev/null 2>&1
sleep 2
kill -INT "$NODE" 2>/dev/null; sleep 1; pkill -9 -x spliner_node
echo "done: $OUT  ($(wc -l < "$OUT") lines, trace $(grep -c '\[trace\]' "$OUT"))"
