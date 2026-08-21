#!/usr/bin/env zsh
# 시뮬 시나리오 실행기 — 저장소의 ./run_sim.sh 를 그대로 쓴다 (PLAN.md §5).
#   usage: DUR=<sec> [POST=<cmd>] [MAP=my_track0820-4] run_sim.zsh <out_prefix> [launch 인자...]
#
# ⚠️ 직접 sim_race.launch.py 를 띄우지 말 것. 그쪽은 **detector 도 control 도 안 띄운다**
#    (launch 주석: "detector/control 은 integration_pkg stack.launch.py 가 별도로 띄운다").
#    한 번 그렇게 돌려서 /perception/obstacles 가 663 프레임 내내 비어 있었고,
#    파이썬 트래커의 tracker/speed_scale 0.30 때문에 랩이 23 s(실차 7.5 s)였다.
#    ./run_sim.sh 는 stack.launch.py 를 띄우고 워크스페이스 5개 source·고아 노드
#    정리·gym 경로 해석까지 대신한다.
setopt noequals
OUT=$1; shift
DUR=${DUR:-180}
WS=/home/sangbum/IFAC_ws
MAP=${MAP:-my_track0820-4}

source /opt/ros/humble/setup.zsh
for w in planning perception control localization integration; do
    source $WS/${w}_ws/install/setup.zsh
done
setopt NULL_GLOB; rm -rf $OUT $OUT.*.log; unsetopt NULL_GLOB

cleanup() {
    [[ -n $REC_PID ]] && kill -INT $REC_PID 2>/dev/null
    sleep 3
    [[ -n $LAUNCH_PID ]] && kill -INT $LAUNCH_PID 2>/dev/null
    sleep 4
    # run_sim.sh 와 같은 목록 (고아가 남으면 다음 실행이 토픽 혼선으로 망가진다)
    pkill -9 -f 'gym_bridge|particle_filter|sim_odom_drift_node|virtual_obstacle_node|opponent_driver_node|opponent_predictor_node|waypoint_tracker_node|control_node|obstacle_detector_node|perception_obstacle_bridge_node|frenet_frame_node|local_trajectory_node|spliner_node|global_trajectory_node|recovery_supervisor_node|planner_visualizer_node|integration_node|stack.launch.py' 2>/dev/null
    return 0
}
trap 'cleanup; exit 130' INT TERM

$WS/run_sim.sh $MAP --no-rviz "$@" > $OUT.launch.log 2>&1 &
LAUNCH_PID=$!
echo "stack 기동 대기(25s)..."; sleep 25

ros2 bag record -o $OUT \
    --regex '/pitwall/.*|/planner/.*|/perception/.*|/car_state/.*|/global_waypoints.*|/ackermann_cmd|/drive|/odom|/pf/pose/odom|/scan|/scan_overlay|/map|/tf.*|/rosout|/opponent_prediction/.*|/sim/.*' \
    > $OUT.rec.log 2>&1 &
REC_PID=$!
sleep 3
if [[ -n $POST ]]; then
    echo "POST: $POST"
    eval "$POST" >> $OUT.post.log 2>&1
fi
echo "기록 ${DUR}s"
sleep $DUR
cleanup
echo "done: $OUT"
