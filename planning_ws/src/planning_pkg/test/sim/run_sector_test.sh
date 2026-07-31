#!/bin/bash
# 사용법: run_sector_test.sh [Spielberg_map|Budapest_map]
MAP=${1:-Spielberg_map}
SCRATCH=$(cd "$(dirname "$0")" && pwd)
WS_ROOT=$(cd "$SCRATCH/../../../../.." && pwd)
SIM_WS=${SIM_WS:-"$(dirname "$WS_ROOT")/sim_ws"}
source /opt/ros/humble/setup.bash
source "$SIM_WS/install/setup.bash"
source "$WS_ROOT/planning_ws/install/setup.bash"

cleanup() {
  if [ -n "${LP:-}" ] && kill -0 "$LP" 2>/dev/null; then
    kill -TERM -- "-$LP" 2>/dev/null || true
    # launch가 SIGTERM을 무시하는 자식을 남겨도 테스트 cleanup이 무한 대기하지
    # 않도록, 우리가 만든 process group만 제한 시간 뒤 강제 종료한다.
    for _ in $(seq 1 50); do
      kill -0 "$LP" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$LP" 2>/dev/null; then
      kill -KILL -- "-$LP" 2>/dev/null || true
    fi
    wait "$LP" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM
setsid ros2 launch planning_pkg sim_race.launch.py rviz:=false map:="$MAP" \
  > "/tmp/sim_$MAP.log" 2>&1 &
LP=$!
sleep 12
python3 "$SCRATCH/sector_test.py" "$MAP"
RC=$?
cleanup
LP=
trap - EXIT INT TERM
exit $RC
