#!/usr/bin/env zsh
# spliner_node 단독 재생 — 백의 인지/측위 출력을 그대로 먹여 카운터를 다시 뽑는다.
#   usage: [DUR=<sec>] replay_spliner.zsh <bag_dir> <out_prefix>
#
# ⚠️ RATE 인자를 두지 않는다. spliner 의 루프는 create_wall_timer(벽시계)이고
#    staleness 판정은 use_sim_time(=재생시계)이라, 배속을 주면 sim 초당 사이클
#    수가 배속만큼 줄어 카운터가 통째로 어긋난다. 항상 1배속.
#
# 개루프다: 자차 궤적은 백의 것을 그대로 쓰므로 spliner 의 판단이 차를 움직이지
# 않는다. P0 처럼 거동 변경이 없는 계측을 검증할 때만 유효하다.
BAG=$1; OUT=$2; DUR=${DUR:-0}
# 워크스페이스 경로는 스크립트 위치에서 되짚는다 — 예전엔 하드코딩이라
# 다른 머신(홈 디렉터리가 다름)에서 조용히 '백 없음'으로 끝났다.
WS=${WS:-${0:A:h:h:h}}
[[ -d $BAG ]] || { echo "bag 없음: $BAG" >&2; exit 1 }

# ── 중복 노드 가드 (CLAUDE.md §0) ──────────────────────────────────────────
# ROS 2 는 같은 이름 노드가 둘 떠도 에러를 안 낸다. 둘 다 /pitwall/* 에 쓰면
# 카운터가 뒤섞여 측정을 통째로 날린다. 실제로 한 번 날렸다.
# pgrep -f 는 **호출한 쉘의 명령줄까지** 훑어서 자기 자신을 잡는다(실제로 잡았다).
# 프로세스 이름 정확매칭(-x)을 쓴다 — ros2 run 래퍼는 python3 라 안 걸리고
# 실제 노드 바이너리만 걸린다.
if pgrep -x spliner_node > /dev/null 2>&1; then
    echo "이미 spliner_node 가 떠 있다 — 죽이고 다시 실행할 것:" >&2
    pgrep -ax spliner_node >&2
    exit 1
fi

source /opt/ros/humble/setup.zsh
source $WS/planning_ws/install/setup.zsh
SHARE=$WS/planning_ws/install/planning_pkg/share/planning_pkg
setopt NULL_GLOB; rm -rf $OUT $OUT.*.log; unsetopt NULL_GLOB

NODE_PID=0; REC_PID=0
cleanup() {
    [[ $REC_PID -ne 0 ]] && kill -INT $REC_PID 2>/dev/null
    sleep 4
    [[ $NODE_PID -ne 0 ]] && kill -INT $NODE_PID 2>/dev/null
    sleep 2
    pkill -x spliner_node 2>/dev/null
    pkill -f "bag record -o $OUT" 2>/dev/null
    return 0
}
# set -e 를 쓰지 않는다 — 예전 판에서 kill 의 비영(非零) 반환이 스크립트를
# 중간에 끊어 노드가 살아남았고, 다음 실행과 겹쳐 위 사고가 났다.
trap 'cleanup; exit 130' INT TERM

ros2 run planning_pkg spliner_node --ros-args \
  --params-file $SHARE/config/params.yaml \
  --params-file $SHARE/config/environments/real_car.yaml \
  -p use_sim_time:=true \
  -p spliner/obstacle_topic:=/perception/obstacles_frenet \
  > $OUT.node.log 2>&1 &
NODE_PID=$!
sleep 3

# pitwall::Log 는 첫 호출에 퍼블리셔를 만들므로 토픽이 재생 중에 늘어난다.
# 고정 목록 대신 regex 로 잡는다.
ros2 bag record -o $OUT --regex '/pitwall/.*|/planner/.*|/rosout' \
  > $OUT.rec.log 2>&1 &
REC_PID=$!
sleep 3

PLAY="ros2 bag play $BAG --clock 200 --topics \
  /perception/obstacles_frenet /opponent_prediction/obstacles \
  /car_state/frenet/odom /global_waypoints /tf /tf_static /map"
if [[ $DUR -gt 0 ]]; then timeout $DUR ${=PLAY} > $OUT.play.log 2>&1
else ${=PLAY} > $OUT.play.log 2>&1; fi

sleep 3
cleanup
echo "done: $OUT"
