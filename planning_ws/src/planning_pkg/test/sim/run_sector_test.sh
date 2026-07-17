#!/bin/bash
# 사용법: run_sector_test.sh [Spielberg_map|Budapest_map]
MAP=${1:-Spielberg_map}
SCRATCH=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/humble/setup.bash
source /home/sangbum/sim_ws/install/setup.bash
source /home/sangbum/IFAC_ws/planning_ws/install/setup.bash
# 주의: 이 스크립트 자신이 src/planning_pkg/ 아래 있으므로 "planning_pkg"로
# 잡으면 자기 자신을 죽인다(exit 144). 노드 실행파일은 install/planning_pkg/
# 아래에만 있으니 그걸로 구분한다.
PAT="gym_bridg[e]|install/planning_pk[g]|sim_race.launc[h]|map_serve[r]|lifecycle_manage[r]|robot_state_publishe[r]"

cleanup() {
  kill $(pgrep -f "$PAT") 2>/dev/null
  for i in $(seq 1 20); do
    pgrep -f "$PAT" >/dev/null || break
    sleep 0.5
  done
  kill -9 $(pgrep -f "$PAT") 2>/dev/null
  sleep 1
}

cleanup
ros2 launch planning_pkg sim_race.launch.py rviz:=false map:=$MAP \
  > "/tmp/sim_$MAP.log" 2>&1 &
LP=$!
sleep 12
python3 "$SCRATCH/sector_test.py" "$MAP"
RC=$?
kill $LP 2>/dev/null
sleep 2
cleanup
exit $RC
