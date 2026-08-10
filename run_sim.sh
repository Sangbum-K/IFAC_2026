#!/usr/bin/env zsh
# gym 시뮬 실행기 (워크스페이스 source 순서까지 대신 해준다)
#
# 맵 이름은 **필수**다 — 기본 맵을 두면 실차 트랙이 바뀔 때 옛 트랙 위에서
# 조용히 돈다. CSV 생성·빌드까지 한 번에 하려면 ./simm.sh <맵이름> 을 쓸 것.
#
#   ./run_sim.sh <맵이름>              실차 최대 근사 — env:=sim_real_car
#                                      (PF + 실차 차량물리 + 실차 파라미터)
#   ./run_sim.sh <맵이름> --gt         위치추정을 gym GT로 (pf:=false, 빠른 확인용)
#   ./run_sim.sh <맵이름> --wall-filter 브릿지 맵 벽거리 필터 켜기 (map_filter:=true)
#   ./run_sim.sh <맵이름> --plain      기존 sim 프로파일 (env:=sim, gym 기본 물리)
#   ./run_sim.sh <맵이름> --no-rviz    RViz 없이
#   ./run_sim.sh <맵이름> suffix:=...  그 외 인자는 ros2 launch로 그대로 전달
#
# 빌드가 필요하면 먼저: ./build_all.sh sim   (gym은 $GYM_WS에서 따로 빌드)

# zsh의 equals 확장(=cmd -> 경로)이 rviz:=false 같은 launch 인자를 망가뜨린다.
setopt noequals

ROOT=${0:A:h}
GYM_WS=${GYM_WS:-$HOME/sim_ws}

fail() { print -P "%F{red}✗ $1%f"; exit 1 }
info() { print -P "%F{cyan}▶ $1%f" }
warn() { print -P "%F{yellow}⚠ $1%f" }

env_arg=sim_real_car
map_arg=
extra=()
for a in "$@"; do
  case $a in
    --gt)           extra+=("pf:=false") ;;
    --wall-filter)  extra+=("map_filter:=true") ;;
    --plain)        env_arg=sim ;;
    --no-rviz)      extra+=("rviz:=false") ;;
    -h|--help)      awk 'NR>1 && /^#/ {sub(/^# ?/,""); print; next} NR>1 {exit}' $0
                    exit 0 ;;
    env:=*)         env_arg=${a#env:=} ;;
    map:=*)         map_arg=${a#map:=} ;;
    --*)            extra+=("$a") ;;
    *:=*)           extra+=("$a") ;;
    *)              # 이름만 준 것 = 맵 이름 (첫 번째만)
                    if [[ -z $map_arg ]]; then map_arg=$a; else extra+=("$a"); fi ;;
  esac
done

if [[ -z $map_arg ]]; then
  fail "맵 이름이 필요합니다 (기본 맵 없음).
  사용법: $0 <맵이름> [옵션]
  맵 목록: ls ${0:A:h}/maps/*.yaml
  CSV 생성+빌드까지 한 번에: ${0:A:h}/simm.sh <맵이름>"
fi

# ── 중복/고아 노드 정리 ────────────────────────────────────────────
# ROS 2는 같은 이름 노드가 둘 떠도 에러를 안 낸다. 남아 있으면 서로 다른 값을
# 같은 토픽에 써서 원인 못 찾는 이상 동작이 난다 (CLAUDE.md 실차 사고 기록과 동일).
#
# 2026-08-07 시뮬에서 실제로 당한 사고: launch가 Ctrl-C 없이 죽으면
# sim_odom_drift_node / gym_bridge 가 고아로 남는다. 다음 실행에서 드리프트 odom이
# 두 벌 나가 /odom·/tf 타임스탬프가 뒤엉키고, PF 보정이 jump guard를 넘겨
# `Rejected PF scan update ... exceeds guard` 가 폭주하며 위치추정이 발산했다
# (경로오차 85~240 m). 옛 패턴은 gym_bridge|particle_filter|control_node 만 봐서
# 정작 원인인 drift 노드를 놓쳤고, 물어보는 프롬프트라 비대화식 실행에선 그냥 넘어갔다.
# 시뮬 프로세스는 언제든 다시 띄우면 되므로 묻지 않고 정리한다.
SIM_NODES='gym_bridge|particle_filter|sim_odom_drift_node|virtual_obstacle_node|opponent_driver_node|opponent_predictor_node|waypoint_tracker_node|control_node|obstacle_detector_node|perception_obstacle_bridge_node|frenet_frame_node|local_trajectory_node|spliner_node|global_trajectory_node|recovery_supervisor_node|planner_visualizer_node|integration_node'
leftovers=(${(f)"$(pgrep -f $SIM_NODES 2>/dev/null)"})
if (( ${#leftovers} )); then
  warn "이전 시뮬 프로세스 ${#leftovers}개가 남아 있어 정리합니다 (중복 노드 = 토픽 혼선)."
  for pid in $leftovers; do kill $pid 2>/dev/null; done
  sleep 1
  for pid in ${(f)"$(pgrep -f $SIM_NODES 2>/dev/null)"}; do kill -9 $pid 2>/dev/null; done
  info "정리 완료"
fi

# ── source (zsh에서 setup.bash는 깨진다 — setup.zsh만 쓸 것) ────────
source /opt/ros/humble/setup.zsh || fail "ROS 2 Humble source 실패"

[[ -f $GYM_WS/install/setup.zsh ]] || fail \
  "gym 워크스페이스가 없습니다: $GYM_WS
  f1tenth_gym_ros를 거기서 빌드하거나 GYM_WS=<경로> 로 지정하세요."
source $GYM_WS/install/setup.zsh

# 순서 중요: planning이 메시지 패키지를 갖고 있어 먼저.
# localization은 particle_filter (pf:=true 경로) 때문에 필요.
for ws in planning perception control localization integration; do
  [[ -f $ROOT/${ws}_ws/install/setup.zsh ]] || fail \
    "${ws}_ws가 빌드되지 않았습니다 — 먼저 ./build_all.sh sim 을 실행하세요"
  source $ROOT/${ws}_ws/install/setup.zsh
done

info "map:=$map_arg env:=$env_arg ${extra[*]}"
if [[ $env_arg == sim_real_car ]]; then
  print "  실차 근사: particle_filter / mu 0.75·a_max 3.5·조향 ±0.36 / LiDAR 40 Hz"
  print "  ⓘ 벽이 장애물로 잡혀 차가 출발 안 할 수 있습니다 (실차 08-04와 같은 현상)."
fi

exec ros2 launch integration_pkg stack.launch.py \
  "env:=$env_arg" "map:=$map_arg" "${extra[@]}"
