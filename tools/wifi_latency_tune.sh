#!/usr/bin/env bash
# wifi_latency_tune.sh — ROS 2 원격 모니터링용 WiFi 지연 튜닝 (머신당 1회, root)
#
# 출처: UNICORN Racing, "WiFi Latency Reduction for ROS2"
#   https://hmcl-unist.github.io/unicorn-racing/posts/wifi-latency-tuning/
# 원문은 ping max 39 ms → 3 ms 를 보고한다. 세 갈래를 건드린다:
#
#   ① 커널 소켓 버퍼 상한 (net.core.{r,w}mem_max)
#      기본 208 KB. DDS 는 참가자당 ~1 MB 를 요청하므로 그대로 두면 커널이
#      조용히 깎고, LiDAR PointCloud2 처럼 큰 메시지가 버스트로 나갈 때 드롭된다.
#      우리 스택은 CycloneDDS 가 아니라 FastDDS(기본 RMW)를 쓰지만, FastDDS 도
#      같은 UDP 소켓 버퍼를 쓰므로 이 항목은 그대로 유효하다.
#      2147483647 은 "상한"이지 예약이 아니다 — 실제 메모리를 먹지 않는다.
#
#   ② 802.11 파워세이브 (NetworkManager 기본값)
#      우분투 기본 wifi.powersave=3(켬). 유휴 시 라디오를 재우므로 다음 패킷이
#      DTIM 주기만큼 밀린다 → 수십 ms ping 스파이크. 2 = 끔.
#
#   ③ 드라이버 레벨 저전력 상태
#      원문은 Intel(iwlwifi) 기준이지만 이 차는 Jetson + Realtek RTL8822CE
#      (rtl8822ce 모듈)다. 대응 파라미터로 옮겨 놨다. 둘 다 자동 판별한다.
#
# 사용법:
#   sudo ./tools/wifi_latency_tune.sh                 # 설정 + 즉시 적용
#   sudo ./tools/wifi_latency_tune.sh --reload-driver # 드라이버 모듈까지 재적재(링크 끊김)
#   ./tools/wifi_latency_tune.sh --check              # 현재 상태만 확인 (root 불필요)
#
# ③ 은 모듈 재적재나 재부팅 전까지 적용되지 않는다. 주행 중에 링크를 끊지
# 않으려고 기본값은 "설정만 하고 재부팅 때 반영"이다.

set -u

CHECK_ONLY=0
RELOAD_DRIVER=0
IFACE=""
for a in "$@"; do
  case "$a" in
    --check)         CHECK_ONLY=1 ;;
    --reload-driver) RELOAD_DRIVER=1 ;;
    -h|--help)       sed -n '2,32p' "$0"; exit 0 ;;
    -*)              echo "알 수 없는 옵션: $a" >&2; exit 2 ;;
    *)               IFACE="$a" ;;
  esac
done

red()  { printf '\033[31m%s\033[0m\n' "$*"; }
grn()  { printf '\033[32m%s\033[0m\n' "$*"; }
cyn()  { printf '\033[36m▶ %s\033[0m\n' "$*"; }
warn() { printf '\033[33m! %s\033[0m\n' "$*"; }

# ── WiFi 인터페이스 판별 ────────────────────────────────────────────────
if [ -z "$IFACE" ]; then
  IFACE=$(iw dev 2>/dev/null | awk '$1=="Interface"{print $2; exit}')
fi
if [ -z "$IFACE" ]; then
  red "✗ WiFi 인터페이스를 못 찾았다. 인자로 직접 넘겨라: $0 wlan0"
  exit 1
fi
DRIVER=$(basename "$(readlink -f "/sys/class/net/$IFACE/device/driver" 2>/dev/null)" 2>/dev/null)

status() {
  cyn "현재 상태 (iface=$IFACE, driver=${DRIVER:-unknown})"
  printf '  net.core.rmem_max = %s\n' "$(sysctl -n net.core.rmem_max)"
  printf '  net.core.wmem_max = %s\n' "$(sysctl -n net.core.wmem_max)"
  printf '  %s\n' "$(iw dev "$IFACE" get power_save 2>/dev/null || echo 'Power save: (조회 실패)')"
  local nm; nm=$(grep -rh '^\s*wifi.powersave' /etc/NetworkManager/conf.d/ 2>/dev/null | tail -1)
  printf '  NetworkManager: %s\n' "${nm:-(설정 없음 → 기본 3=켬)}"
  case "$DRIVER" in
    rtl8822ce|rtl88x2ce|rtw_*)
      for p in rtw_power_mgnt rtw_ips_mode rtw_lps_chk_by_tp; do
        [ -f "/sys/module/$DRIVER/parameters/$p" ] &&
          printf '  %s.%s = %s\n' "$DRIVER" "$p" "$(cat "/sys/module/$DRIVER/parameters/$p")"
      done ;;
    iwlwifi)
      [ -f /sys/module/iwlwifi/parameters/power_save ] &&
        printf '  iwlwifi.power_save = %s\n' "$(cat /sys/module/iwlwifi/parameters/power_save)"
      for m in iwlmvm iwlmld; do
        [ -f "/sys/module/$m/parameters/power_scheme" ] &&
          printf '  %s.power_scheme = %s\n' "$m" "$(cat "/sys/module/$m/parameters/power_scheme")"
      done ;;
  esac
}

if [ "$CHECK_ONLY" = 1 ]; then status; exit 0; fi
if [ "$(id -u)" != 0 ]; then
  red "✗ root 로 실행해야 한다:  sudo $0"
  echo "   (읽기만 하려면: $0 --check)"
  exit 1
fi

# ── ① 커널 소켓 버퍼 상한 ───────────────────────────────────────────────
cyn "① 소켓 버퍼 상한 → /etc/sysctl.d/60-cyclonedds.conf"
cat > /etc/sysctl.d/60-cyclonedds.conf << 'EOF'
# ROS 2 DDS 소켓 버퍼 상한 (tools/wifi_latency_tune.sh 가 생성)
# 기본 208 KB 는 DDS 가 요청하는 ~1 MB 에 못 미쳐 대용량 메시지에서 드롭이 난다.
# 상한값일 뿐이라 실제 메모리를 예약하지 않는다.
net.core.rmem_max=2147483647
net.core.wmem_max=2147483647
EOF
sysctl --system > /dev/null && grn "  ✓ 적용됨"

# ── ② NetworkManager 파워세이브 끄기 ────────────────────────────────────
# 파일명 주의: NM 은 conf.d 를 알파벳 순으로 읽고 뒤가 이긴다.
# 우분투 기본 default-wifi-powersave-on.conf(=3) 보다 뒤에 와야 하므로
# 'w'로 시작하는 이 이름을 그대로 써라. '99-' 접두사를 붙이면 오히려 진다.
cyn "② WiFi 파워세이브 끄기 → /etc/NetworkManager/conf.d/wifi-powersave-off.conf"
cat > /etc/NetworkManager/conf.d/wifi-powersave-off.conf << 'EOF'
# 2 = disable. 기본값 3(=enable)이 유휴 시 라디오를 재워 ping 스파이크를 만든다.
[connection]
wifi.powersave = 2
EOF
# 즉시 반영: 프로토콜 레벨은 iw 로 바로 끌 수 있다 (NM 재시작 없이).
iw dev "$IFACE" set power_save off 2>/dev/null && grn "  ✓ $IFACE power_save off (즉시)"
# NM 재시작은 링크를 잠깐 끊을 수 있어 기본으로 하지 않는다. 위 iw 로 이미
# 지금 세션은 꺼졌고, conf 파일이 다음 부팅·재연결부터 이어받는다.
grn "  ✓ 설정 저장됨 (재연결·재부팅 후에도 유지)"

# ── ③ 드라이버 레벨 ─────────────────────────────────────────────────────
cyn "③ 드라이버 레벨 저전력 상태 (driver=${DRIVER:-unknown})"
MODFILE=""
case "$DRIVER" in
  rtl8822ce|rtl88x2ce|rtw_*)
    # Realtek: rtw_power_mgnt(LPS) / rtw_ips_mode(유휴 시 RF off) /
    # rtw_lps_chk_by_tp(트래픽 보고 LPS 재진입) 셋 다 꺼야 지연이 안정된다.
    MODFILE=/etc/modprobe.d/${DRIVER}.conf
    cat > "$MODFILE" << EOF
# Realtek WiFi 저전력 상태 끄기 — ROS 2 지연 안정화 (tools/wifi_latency_tune.sh)
# rtw_power_mgnt=0  : 802.11 LPS 끔 (기본 2)
# rtw_ips_mode=0    : 유휴 시 RF 전원 차단(IPS) 끔 (기본 1)
# rtw_lps_chk_by_tp=0: 처리량 보고 LPS 로 되돌아가는 동작 끔 (기본 1)
options $DRIVER rtw_power_mgnt=0 rtw_ips_mode=0 rtw_lps_chk_by_tp=0
EOF
    ;;
  iwlwifi)
    # 원문 그대로. 커널 6.17+ 는 iwlmvm 대신 iwlmld 를 쓴다.
    KMAJ=$(uname -r | cut -d. -f1); KMIN=$(uname -r | cut -d. -f2)
    if [ "$KMAJ" -gt 6 ] || { [ "$KMAJ" = 6 ] && [ "$KMIN" -ge 17 ]; }; then SUB=iwlmld; else SUB=iwlmvm; fi
    MODFILE=/etc/modprobe.d/iwlwifi-ros2.conf
    cat > "$MODFILE" << EOF
options iwlwifi power_save=0
options $SUB power_scheme=1
EOF
    ;;
  *)
    warn "지원 목록에 없는 드라이버라 ③ 은 건너뛴다. 프로토콜 레벨(②)은 적용됨."
    ;;
esac

if [ -n "$MODFILE" ]; then
  grn "  ✓ $MODFILE 작성"
  if [ "$RELOAD_DRIVER" = 1 ]; then
    warn "  드라이버 재적재 — 링크가 잠깐 끊긴다"
    modprobe -r "$DRIVER" && modprobe "$DRIVER" && grn "  ✓ $DRIVER 재적재됨"
  else
    warn "  모듈 파라미터는 재부팅(또는 --reload-driver) 후에 반영된다"
  fi
fi

echo
status
