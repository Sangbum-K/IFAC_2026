#!/usr/bin/env zsh
# 고정 원통 대역 A/B. 사용: replay_bridge_ab.zsh <old|new> <출력이름>
#   old = 2026-08-22 이전 값 (radius 0.25, band [0.05,0.25])
#   new = yaml 현재 값 (radius 0.30, band [0.02,0.44])
set +u
MODE="$1"; TAG="$2"
BAG=/home/miru/IFAC_ws/run_0822_192752
SHARE=/home/miru/IFAC_ws/planning_ws/install/planning_pkg/share/planning_pkg/config
DIR=/home/miru/IFAC_ws/tools/bag_analysis_0822d
OUT=$DIR/frenet_${TAG}

source /opt/ros/humble/setup.zsh
source /home/miru/IFAC_ws/planning_ws/install/setup.zsh

pkill -9 -f perception_obstacle_bridge_node.py 2>/dev/null; sleep 1
rm -rf "$OUT"

OVERRIDE=()
if [[ "$MODE" == "old" ]]; then
  OVERRIDE=(--params-file "$DIR/old_cylinder.yaml")
fi

ros2 run planning_pkg perception_obstacle_bridge_node.py --ros-args \
  --params-file "$SHARE/params.yaml" \
  --params-file "$SHARE/environments/real_car.yaml" \
  "${OVERRIDE[@]}" -p use_sim_time:=true > "$DIR/bridge_${TAG}.log" 2>&1 &
sleep 4
ros2 bag record -o "$OUT" /perception/obstacles_frenet > /dev/null 2>&1 &
REC=$!
sleep 2
ros2 bag play "$BAG" --clock 200 --rate 3 --topics \
  /perception/obstacles /global_waypoints /tf /tf_static /map > /dev/null 2>&1
sleep 2
kill -INT "$REC" 2>/dev/null; sleep 3
pkill -9 -f perception_obstacle_bridge_node.py 2>/dev/null
echo "done: $OUT"
grep -c "" "$DIR/bridge_${TAG}.log" > /dev/null
tail -3 "$DIR/bridge_${TAG}.log"
