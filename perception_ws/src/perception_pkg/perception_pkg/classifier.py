"""
⑤ 정적/동적 판별 — 속도 + 위치분산 + 지속성(상대 정체성)

속도(KF speed)만으로는 두 가지를 놓친다:
  (a) 코너 감속/정차로 느려진 상대를 정적으로 오분류,
  (b) KF 속도 노이즈로 정적 장애물이 동적으로 깜빡임.

그래서 map 프레임 "위치 분산"을 함께 본다 — 진짜 정적 물체는 여러 프레임에
걸쳐 위치가 거의 안 변하고(분산 ~0), 움직이면 변위가 쌓인다. 속도가 임계값
아래여도 위치 분산이 크면 동적으로 본다(느리게 꾸준히 움직이는 상대).

그리고 한 번 동적으로 확정된 트랙(= 상대 차량)은 잠깐 멈춰도 오래 유지한다
(ever_dynamic -> 긴 release). H2H에서 상대는 딱 1대이므로 정체성을 지켜야
GP 예측 버퍼와 회피 로직이 끊기지 않는다.

깜빡임 방지용 히스테리시스: "동적 확정 = 연속 n_confirm 프레임", "정적 복귀 =
연속 release 프레임" (양방향 모두 연속 조건).
"""
import numpy as np


def _pos_spread(pos_hist):
    """측정 위치 이력의 공간 퍼짐 [m] = x/y 표준편차 중 큰 값. 표본 부족이면 None."""
    if pos_hist is None or len(pos_hist) < 3:
        return None
    arr = np.asarray(pos_hist, dtype=float)
    return float(max(arr[:, 0].std(), arr[:, 1].std()))


def classify_track(track, v_thresh=0.25, n_confirm=5, n_release=8,
                   static_pos_std=0.08, dynamic_pos_std=0.20,
                   dynamic_release=40):
    """
    track.is_dynamic / dynamic_streak / static_streak / ever_dynamic / confidence 갱신.

    Parameters
    ----------
    v_thresh        : 동적 판별 속도 임계값 [m/s]
    n_confirm       : 동적으로 '확정'되기까지 필요한 연속 프레임 수
    n_release       : (일반 트랙) 정적으로 되돌아가기까지 필요한 연속 저속 프레임 수
    static_pos_std  : 위치 분산이 이 미만이면 '정적' 근거 [m]
    dynamic_pos_std : 위치 분산이 이 초과이면 (저속이어도) '동적' 근거 [m]
    dynamic_release : 한 번 동적이었던(상대) 트랙이 정적으로 되돌아가기까지
                      필요한 연속 저속·저분산 프레임 수 (일반보다 훨씬 길게 = 정체성 유지)
    """
    speed = track.kf.speed          # 칼만필터가 추정한 현재 속도 크기
    spread = _pos_spread(track.pos_hist)

    # 프레임 투표:
    #  moving = 속도가 임계 이상 '또는' 위치 분산이 큼 (느리지만 꾸준히 이동)
    #  still  = 저속 '그리고' 위치 분산이 작음 (표본 부족이면 분산 조건은 통과로 봄)
    moving = (speed >= v_thresh) or (spread is not None and spread >= dynamic_pos_std)
    still = (speed < v_thresh) and (spread is None or spread < static_pos_std)

    if moving:                      # 움직임 근거 프레임
        track.dynamic_streak += 1
        track.static_streak = 0
    elif still:                     # 정지 근거 프레임
        track.static_streak += 1
        track.dynamic_streak = 0
    # 모호(저속인데 분산이 중간)한 프레임은 두 streak 모두 유지 -> 깜빡임 방지

    # 정체성: 한 번 상대(동적)로 확정된 트랙은 정차/코너 감속에 쉽게 안 뒤집히게
    # release를 크게 잡는다. 일반 트랙(노이즈 클러스터 등)은 기존 n_release.
    release = dynamic_release if track.ever_dynamic else n_release

    if not track.is_dynamic and track.dynamic_streak >= n_confirm:
        track.is_dynamic = True     # 동적으로 확정
        track.ever_dynamic = True   # 상대 정체성 각인 (이후 잘 안 풀림)
    elif track.is_dynamic and track.static_streak >= release:
        track.is_dynamic = False    # 충분히 오래 정지 -> 정적으로 복귀

    # confidence: 확정 조건 대비 진행률 (0~1). 로깅/디버깅 및 msg.confidence용
    if track.is_dynamic:
        track.confidence = min(1.0, track.dynamic_streak / max(n_confirm, 1))
    elif track.hits > 1:            # 최소 2번 이상 관측된 트랙만 신뢰도 부여
        track.confidence = min(1.0, track.static_streak / max(release, 1))
    else:
        track.confidence = 0.0

    return track.is_dynamic
