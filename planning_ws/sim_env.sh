#!/usr/bin/env bash
# F1TENTH 시뮬 환경 source 헬퍼.
#   source ~/IFAC_ws/planning_ws/sim_env.sh
# 올바른 순서로 ROS/sim/planning/control 워크스페이스를 source 한다.
# (순서가 꼬이면 노드가 'Waiting for global waypoints...'에서 멈춘다 — README §6)

_ros_setup=/opt/ros/humble/setup.bash
_sim_ws=~/sim_ws/install/setup.bash
_planning_ws=~/IFAC_ws/planning_ws/install/setup.bash
_control_ws=~/IFAC_ws/control_ws/install/setup.bash   # 선택 (tracker:=false 시)

for f in "$_ros_setup" "$_sim_ws" "$_planning_ws" "$_control_ws"; do
    if [ -f "$f" ]; then
        source "$f"
    else
        echo "sim_env.sh: 건너뜀 (없음) — $f" >&2
    fi
done

echo "sim_env.sh: sourced. 실행 예:"
echo "  ros2 launch planning_pkg sim_race.launch.py map:=Budapest_map"
