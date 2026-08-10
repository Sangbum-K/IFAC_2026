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

/// track.is_dynamic / dynamic_streak / static_streak / ever_dynamic / confidence 갱신.
///
/// v_thresh        : 동적 판별 속도 임계값 [m/s]
/// n_confirm       : 동적으로 '확정'되기까지 필요한 연속 프레임 수
/// n_release       : (일반 트랙) 정적으로 되돌아가기까지 필요한 연속 저속 프레임 수
/// static_pos_std  : 위치 분산이 이 미만이면 '정적' 근거 [m]
/// dynamic_pos_std : 위치 분산이 이 초과이면 (저속이어도) '동적' 근거 [m]
/// dynamic_release : 한 번 동적이었던(상대) 트랙이 정적으로 되돌아가기까지 필요한
///                   연속 저속·저분산 프레임 수 (일반보다 훨씬 길게 = 정체성 유지)
bool classify_track(
  Track & track,
  double v_thresh, int n_confirm, int n_release,
  double static_pos_std, double dynamic_pos_std, int dynamic_release);

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG__CLASSIFIER_HPP_
