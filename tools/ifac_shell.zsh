# ifac_shell.zsh — IFAC_ws 작업 명령 (sa / rc / rckill / tune / mkcsv)
#
# 설치 (기계당 1회, ~/.zshrc **맨 끝**에 한 줄):
#
#     source ~/IFAC_ws/tools/ifac_shell.zsh
#
# ⚠️ **맨 끝**이어야 한다. zsh 는 같은 이름이면 alias 가 함수보다 먼저 잡히고,
#    나중에 정의된 쪽이 이긴다. 예전 `alias rc=...` 줄이 이 아래에 남아 있으면
#    그 alias 가 계속 쓰인다 — 옛 줄은 지우거나 주석 처리할 것. (이 파일도
#    아래에서 같은 이름의 alias 를 먼저 걷어내지만, 순서까지는 못 고친다.)
#
# 정의를 dotfile 이 아니라 리포지토리에 두는 이유: 기계를 옮길 때마다 손으로
# 다시 만들다가 실제로 잃어버렸다 (mkcsv 가 그랬다 — HOW_TO_RUN.md 는 계속
# `mkcsv <맵>` 으로 안내하는데 정작 명령이 없었다). dotfile 에는 위 한 줄만 둔다.
#
# 경로는 이 파일 위치에서 뽑는다. ~/IFAC_ws 로 못 박으면 젯슨(miru)·노트북·
# worktree 에서 서로 다른 트리를 가리키게 된다.

# 이 파일의 위치 -> 워크스페이스 루트 (tools/ 의 부모).
export IFAC_WS="${${(%):-%x}:A:h:h}"

# 같은 이름의 옛 alias 를 먼저 걷어낸다. zsh 에서 alias 는 함수보다 우선이라
# `alias rc=...` 가 남아 있으면 아래 rc() 가 영영 안 불린다.
unalias sa rc rckill tune mkcsv 2>/dev/null

# 전체 워크스페이스 source — 오버레이 순서 고정 (planning 이 메시지라 먼저).
sa() {
  source /opt/ros/humble/setup.zsh
  local ws
  for ws in planning perception control integration hardware localization; do
    [ -f "$IFAC_WS/${ws}_ws/install/setup.zsh" ] \
      && source "$IFAC_WS/${ws}_ws/install/setup.zsh"
  done
}

# ── 실차 원커맨드 ─────────────────────────────────────────────────────
#   rc                       달릴 수 있는 라인 목록
#   rc gotosleep             짧은 이름으로 지정 (뒤에서부터 _ 단위로 맞춘다)
#   rc my_track0824_realfinal_gotosleep rviz:=false
#   rckill                   남아 있는 스택 프로세스 정리
#
# 한 터미널에서 라인을 바꿔 가며 여러 번 돌릴 때 조용히 꼬이는 경로가 셋 있어서
# 넷 다 launch 전에 막는다. 셋 다 "잘못된 CSV 로 달리고 있는데 화면은 정상"이라
# 사람이 못 알아채는 종류다:
#   1) 짧은 이름이 두 라인에 걸리는데 하나를 골라 버리는 것 → 후보를 보여주고 멈춘다
#   2) 앞선 rc 의 노드가 안 죽고 남아 두 global_trajectory_node 가 서로 다른
#      /global_waypoints 를 던지는 것 (ROS 2 는 같은 이름 노드 중복을 에러로 안 냄)
#   3) tune 으로 저장했는데 build 를 안 해서 install 의 옛 CSV 로 달리는 것
# 그리고 launch 직전에 **실제로 열 CSV 파일과 generation_id 를 찍는다** — 화면에
# 남는 그 한 줄이 "무엇으로 달렸나"의 근거다.

_ifac_traj_src() { print -r -- "$IFAC_WS/planning_ws/src/planning_pkg/config/global_trajectory" }
_ifac_traj_ins() { print -r -- "$IFAC_WS/planning_ws/install/planning_pkg/share/planning_pkg/config/global_trajectory" }

# 소스 트리의 <라인>_iqp_*.csv 에서 라인 이름 목록. 소스가 기준인 이유:
# tune 으로 막 저장한 라인도 목록에 나와야 "빌드하세요" 라고 말해 줄 수 있다.
_ifac_lines() {
  local f
  for f in "$(_ifac_traj_src)"/*_iqp_*.csv(N); do
    print -r -- "${${f:t}%%_iqp_*}"
  done | sort -u
}

# 짧은 이름 -> 라인 전체 이름. 정확히 하나로 안 좁혀지면 stdout 에 아무것도 안 쓰고 1.
_ifac_resolve_line() {
  local want="$1" l
  local -a exact suffix
  for l in ${(f)"$(_ifac_lines)"}; do
    [[ "$l" == "$want" ]] && exact+=("$l")
    [[ "$l" == *"_$want" ]] && suffix+=("$l")
  done
  if (( ${#exact} == 1 )); then print -r -- "${exact[1]}"; return 0; fi
  if (( ${#exact} == 0 && ${#suffix} == 1 )); then print -r -- "${suffix[1]}"; return 0; fi
  local -a hits
  hits=(${exact} ${suffix})
  if (( ${#hits} == 0 )); then
    print -u2 "rc: '$want' 에 맞는 라인이 없습니다."
  else
    print -u2 "rc: '$want' 가 ${#hits}개에 걸립니다 — 임의로 고르지 않습니다:"
    for l in ${hits}; do print -u2 "      $l"; done
  fi
  print -u2 ""
  print -u2 "  달릴 수 있는 라인:"
  for l in ${(f)"$(_ifac_lines)"}; do print -u2 "      $l"; done
  return 1
}

# 살아 있는 스택 프로세스 pid. 셸/pgrep 자신은 comm 으로 걸러낸다.
_ifac_stack_pids() {
  local pat='real_car\.launch\.py|stack\.launch\.py|planner_stack\.launch\.py|mapping\.launch\.py|bringup_launch\.py|global_trajectory_node|local_trajectory_node|spliner_node|recovery_supervisor_node|planner_visualizer_node|opponent_predictor_node|perception_obstacle_bridge_node|frenet_frame_node|obstacle_detector_node|obstacle_viz_node|control_node|integration_node|waypoint_tracker_node|wait_for_localization|particle_filter_cpp_node|vesc_driver_node|ackermann_to_vesc|vesc_to_odom|throttle_interpolator|ackermann_mux|urg_node'
  local pid comm
  for pid in ${(f)"$(pgrep -f -- "$pat" 2>/dev/null)"}; do
    [[ -z "$pid" || "$pid" == "$$" ]] && continue
    comm="$(ps -o comm= -p "$pid" 2>/dev/null)"
    case "$comm" in
      zsh|bash|sh|dash|pgrep|grep|ps|awk|sed|"") continue ;;
    esac
    print -r -- "$pid"
  done
}

rckill() {
  local -a pids
  pids=(${(f)"$(_ifac_stack_pids)"})
  if (( ${#pids} == 0 )); then print "rckill: 남아 있는 스택 프로세스 없음"; return 0; fi
  print "rckill: ${#pids}개 종료합니다"
  ps -o pid=,args= -p ${(j:,:)pids} 2>/dev/null | cut -c1-110
  kill -INT ${pids} 2>/dev/null
  local i
  for i in {1..40}; do          # 최대 10초 — 스택 전체가 정상 종료할 시간을 준다
    pids=(${(f)"$(_ifac_stack_pids)"})
    (( ${#pids} == 0 )) && { print "rckill: 정리 완료"; return 0 }
    sleep 0.25
  done
  print -u2 "rckill: INT 로 안 죽어 KILL 합니다 — ${pids}"
  kill -KILL ${pids} 2>/dev/null
  sleep 0.5
  pids=(${(f)"$(_ifac_stack_pids)"})
  (( ${#pids} == 0 )) && print "rckill: 정리 완료" || {
    print -u2 "rckill: 아직 남음 — ${pids}"; return 1 }
}

_ifac_list_lines() {
  local src="$(_ifac_traj_src)" ins="$(_ifac_traj_ins)" l f g state
  print "달릴 수 있는 라인 (소스 트리 기준):"
  for l in ${(f)"$(_ifac_lines)"}; do
    local -a sf
    sf=("$src"/${l}_iqp_*.csv(N))
    if (( ${#sf} != 1 )); then
      state="⛔ 같은 이름 CSV ${#sf}개 — launch 가 거부합니다"
    elif [[ ! -f "$ins/${sf[1]:t}" ]]; then
      state="⚠️  install 에 없음 — ./build_all.sh 필요"
    elif ! cmp -s "${sf[1]}" "$ins/${sf[1]:t}"; then
      state="⚠️  install 이 옛 내용 — ./build_all.sh 필요"
    else
      state="✅ 준비됨"
      g="$(head -1 "$ins/${sf[1]:t}" 2>/dev/null)"
    fi
    printf '  %-42s %s\n' "$l" "$state"
    (( ${#sf} == 1 )) && printf '  %-42s   %s\n' "" "${sf[1]:t}"
  done
  # 이름 규약을 안 따르는 CSV 는 launch 의 `<라인>_iqp_*.csv` 글롭에 안 걸려서
  # rc 로 부를 수 없다. 조용히 빼면 "분명 파일을 넣었는데 목록에 없다" 가 된다.
  local -a odd
  for f in "$src"/*.csv(N); do
    [[ "${f:t}" == *_iqp_* || "${f:t}" == *_sp_* ]] || odd+=("${f:t}")
  done
  if (( ${#odd} > 0 )); then
    print ""
    print "  rc 로 못 부르는 CSV ${#odd}개 (이름에 _iqp_ 가 없어 launch 글롭에 안 걸림):"
    for f in ${odd[1,6]}; do print "      $f"; done
    (( ${#odd} > 6 )) && print "      ... 외 $(( ${#odd} - 6 ))개"
    print "      -> rc <맵> trajectory_csv_file:=$odd[1]  처럼 직접 지정하세요"
  fi
  print ""
  print "  rc <이름>   이름은 뒤에서부터 _ 단위로 맞습니다 (예: rc gotosleep)"
}

# 이 셸이 **이 워크스페이스로** 제대로 소스돼 있는가.
#   · 하나도 안 됐거나 반만 됐으면 sa 를 대신 돌린다 (반쯤 소스된 셸은 오버레이
#     순서가 깨져서, 어떤 패키지는 install 을 어떤 패키지는 /opt/ros 를 본다)
#   · 다른 IFAC_ws 사본(worktree·백업 체크아웃)이 섞여 있으면 **멈춘다**. 그건
#     같은 이름 패키지가 둘이라는 뜻이고, 어느 쪽 CSV 를 여는지 사람이 알 수 없다.
_ifac_env_check() {
  local ws p
  local -a want missing
  for ws in planning perception control integration hardware localization; do
    p="$IFAC_WS/${ws}_ws/install"
    [[ -f "$p/setup.zsh" ]] || continue
    want+=("$p")
    [[ ":$AMENT_PREFIX_PATH:" == *":$p/"* ]] || missing+=("${ws}_ws")
  done
  if (( ${#want} == 0 )); then
    print -u2 "rc: 빌드된 워크스페이스가 하나도 없습니다 — cd $IFAC_WS && ./build_all.sh"
    return 1
  fi
  if (( ${#missing} > 0 )); then
    print "rc: 소스 안 된 워크스페이스(${(j:, :)missing}) — sa 를 먼저 실행합니다"
    sa
  fi
  local -a stray
  for p in ${(s.:.)AMENT_PREFIX_PATH}; do
    [[ "$p" == *IFAC* && "$p" != "$IFAC_WS"/* ]] && stray+=("$p")
  done
  if (( ${#stray} > 0 )); then
    print -u2 "rc: 다른 IFAC_ws 사본이 이 셸에 섞여 있습니다 — 어느 CSV 를 여는지 보장이 안 됩니다:"
    for p in ${stray}; do print -u2 "      $p"; done
    print -u2 "    새 터미널을 열고 sa 부터 다시 하세요."
    return 1
  fi
  return 0
}

rc() {
  if [[ -z "$1" || "$1" == "--list" || "$1" == "-l" ]]; then
    _ifac_list_lines
    [[ -z "$1" ]] && return 2 || return 0
  fi

  local line
  line="$(_ifac_resolve_line "$1")" || return 2
  shift

  _ifac_env_check || return 7

  # 남은 프로세스 — 두 global_trajectory_node 가 동시에 발행하면 화면은 정상인데
  # 차가 두 라인을 번갈아 따라간다. 여기서 죽이지 않고 멈추는 이유는, 다른
  # 터미널에서 일부러 띄운 hw / bagviz 를 말없이 죽이면 안 되기 때문이다.
  local -a leftover
  leftover=(${(f)"$(_ifac_stack_pids)"})
  if (( ${#leftover} > 0 )); then
    print -u2 "rc: 이미 떠 있는 스택 프로세스 ${#leftover}개가 있습니다. 겹쳐 띄우면"
    print -u2 "    같은 이름 노드가 둘이 되어 서로 다른 CSV 를 같은 토픽에 씁니다."
    ps -o pid=,args= -p ${(j:,:)leftover} 2>/dev/null | cut -c1-110 >&2
    print -u2 ""
    print -u2 "    정리:  rckill        (일부러 띄운 거면 그 터미널에서 Ctrl+C)"
    return 3
  fi

  # 소스 <-> install CSV 대조. tune 으로 저장하고 빌드를 안 하면 옛 라인으로 달린다.
  local src="$(_ifac_traj_src)" ins="$(_ifac_traj_ins)"
  local -a sf
  sf=("$src"/${line}_iqp_*.csv(N))
  if (( ${#sf} != 1 )); then
    print -u2 "rc: '$line' 에 맞는 소스 CSV 가 ${#sf}개입니다 (정확히 1개여야 함):"
    local f; for f in ${sf}; do print -u2 "      ${f:t}"; done
    print -u2 "    옛 mu/suffix 파일이 남아 있으면 지우거나 옮기세요."
    return 4
  fi
  local base="${sf[1]:t}"
  if [[ ! -f "$ins/$base" ]]; then
    print -u2 "rc: install 에 '$base' 가 없습니다 — 아직 빌드 안 된 라인입니다."
    print -u2 "    cd $IFAC_WS && ./build_all.sh      (실패하면 ./build_all.sh sim)"
    return 5
  fi
  if ! cmp -s "${sf[1]}" "$ins/$base"; then
    print -u2 "rc: 소스 CSV 와 install CSV 가 다릅니다 — 지금 달리면 **옛 라인**입니다."
    print -u2 "    소스   $(head -1 "${sf[1]}")"
    print -u2 "    install $(head -1 "$ins/$base")"
    print -u2 "    cd $IFAC_WS && ./build_all.sh      (실패하면 ./build_all.sh sim)"
    return 6
  fi

  print "─────────────────────────────────────────────────────────────"
  print "  라인   $line"
  print "  CSV    $ins/$base"
  print "  $(head -1 "$ins/$base")"
  # 맵 yaml 이 그 이름으로 있으면 원본(strict), 없으면 라인 변형(advisory) 이다.
  # launch 가 접미사를 보고 정하는 것과 같은 결론을 목록 없이 데이터로 낸다.
  if [[ -f "$IFAC_WS/maps/$line.yaml" ]]; then
    print "  검증   strict — 마찰원/조향 한계를 넘으면 노드가 거부합니다"
  else
    print "  검증   advisory — 라인 변형이라 물리 위반이 경고로만 남습니다"
  fi
  print "─────────────────────────────────────────────────────────────"
  ros2 launch integration_pkg real_car.launch.py map:="$line" "$@"
}

# 레이스라인 튜너 웹앱 — tune [맵|라인] [--reference <라인>|off] [--port N]
tune() { python3 "$IFAC_WS/tools/racelinetuner.py" "$@" }

# 레이스라인 CSV 생성 — mkcsv <맵> [cw|ccw] [build_map.py 인자...]
mkcsv() {
  if [ -z "$1" ]; then
    print -u2 "사용법: mkcsv <맵> [cw|ccw] [build_map.py 인자...]"
    return 2
  fi
  local map="$1"; shift
  local -a dir
  case "${1:-}" in
    cw|ccw|auto) dir=(--dir "$1"); shift ;;
  esac
  "$IFAC_WS/planning_ws/src/planning_pkg/scripts/global_traj_sim/make_raceline.sh" \
    "$map" "${dir[@]}" "$@"
}
