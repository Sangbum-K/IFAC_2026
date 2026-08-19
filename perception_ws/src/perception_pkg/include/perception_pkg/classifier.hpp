/*
 * ⑤ 정적/동적 판별 — 속도 + 위치분산 + 지속성(상대 정체성)
 *
 * 속도(KF speed)만으로는 두 가지를 놓친다:
 *   (a) 코너 감속/정차로 느려진 상대를 정적으로 오분류,
 *   (b) KF 속도 노이즈로 정적 장애물이 동적으로 깜빡임.
 *
 * 그래서 map 프레임 "위치 분산"을 함께 본다 — 진짜 정적 물체는 여러 프레임에
 * 걸쳐 위치가 거의 안 변하고(분산 ~0), 움직이면 변위가 쌓인다. 속도가 임계값
 * 아래여도 위치 분산이 크면 동적으로 본다(느리게 꾸준히 움직이는 상대).
 *
 * 그리고 한 번 동적으로 확정된 트랙(= 상대 차량)은 잠깐 멈춰도 오래 유지한다
 * (ever_dynamic -> 긴 release). H2H에서 상대는 딱 1대이므로 정체성을 지켜야
 * GP 예측 버퍼와 회피 로직이 끊기지 않는다.
 *
 * 깜빡임 방지용 히스테리시스: "동적 확정 = 연속 n_confirm 프레임", "정적 복귀 =
 * 연속 release 프레임" (양방향 모두 연속 조건).
 */
#ifndef PERCEPTION_PKG__CLASSIFIER_HPP_
#define PERCEPTION_PKG__CLASSIFIER_HPP_

#include "perception_pkg/tracker.hpp"

namespace perception_pkg
{

/// track.is_dynamic / dynamic_time / static_time / ever_dynamic / confidence 갱신.
/// has_measurement 가 false이면 KF 코스팅(외삽) 프레임이다. 이 경우에는 실제
/// 라이다 증거가 없으므로 어떤 시간/분류/신뢰도도 진전시키지 않는다.
///
/// ⚠️ 2026-08-19: 문턱을 **프레임 수에서 초로** 바꿨다. 시간은 트랙의
/// last_meas_dt(직전 측정과의 간격)로 적립하므로, 실측률이 트랙마다 달라도
/// "0.78 초 동안 움직임 근거가 이어지면 확정"이라는 계약이 그대로 지켜진다.
/// 예전에는 노드가 이 문턱을 **발행 주기**로 프레임 환산했는데 시간은
/// **측정 프레임**에서만 흘러서, 실측률 33% 이던 run_0818 에서 0.78 초 창이
/// 실제로는 2.33 초로 동작했다 (tracker.hpp Track::dynamic_time 주석 참조).
///
/// v_thresh        : 동적 판별 속도 임계값 [m/s]
/// t_confirm       : 동적으로 '확정'되기까지 필요한 연속 움직임 시간 [s]
/// t_release       : (일반 트랙) 정적으로 되돌아가기까지 필요한 연속 정지 시간 [s]
/// static_pos_std  : 위치 분산이 이 미만이면 '정적' 근거 [m]
/// dynamic_pos_std : 위치 분산이 이 초과이면 (저속이어도) '동적' 근거 [m]
/// t_dynamic_release : 한 번 동적이었던(상대) 트랙이 정적으로 되돌아가기까지
///                   필요한 연속 정지 시간 [s] (일반보다 훨씬 길게 = 정체성 유지)
bool classify_track(
  Track & track, bool has_measurement,
  double v_thresh, double t_confirm, double t_release,
  double static_pos_std, double dynamic_pos_std, double t_dynamic_release);

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG__CLASSIFIER_HPP_
