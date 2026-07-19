"""
⑤ 정적/동적 판별 모듈
- 순간 속도 노이즈로 인한 오분류(깜빡임)를 막기 위해
  "속도 임계값 초과 + 연속 N프레임 유지" 조건을 모두 만족해야 동적으로 확정한다.
- 한 번 동적으로 확정된 트랙은 속도가 임계값 아래로 몇 프레임 떨어져도
  바로 정적으로 바뀌지 않도록 별도의 완화(release) 조건을 둔다 (hysteresis).
"""


def classify_track(track, v_thresh=0.25, n_confirm=5, n_release=8):
    """
    track.kf.speed, track.dynamic_streak, track.is_dynamic을 갱신한다.

    Parameters
    ----------
    v_thresh : 동적 판별 속도 임계값 [m/s]
    n_confirm : 동적으로 '확정'되기까지 필요한 연속 프레임 수
    n_release : 이미 동적인 트랙이 '정적으로 되돌아가기'까지 필요한
                연속 저속 프레임 수 (급감속/노이즈로 인한 깜빡임 방지)
    """
    speed = track.kf.speed #칼만 필터가 추정한 현재 속도 크기를 가져옴

    if speed >= v_thresh: #이번 프레임 속도가 v_tresh(0.25m/s)이상이면 
        track.dynamic_streak += 1
        track.static_streak = 0
    else:
        track.dynamic_streak = 0
        track.static_streak = getattr(track, "static_streak", 0) + 1

    if not track.is_dynamic and track.dynamic_streak >= n_confirm: #정적 상태인데 동적 카운터가 5이상 쌓였다. 동적으로 전환
        track.is_dynamic = True # 동적으로 전환
    elif track.is_dynamic and track.static_streak >= n_release: #동적 상태인데 정적 카운터가 8이상 쌓였다.
        track.is_dynamic = False #정적으로 전환

    # confidence: 최근 확정 조건 대비 진행률 (0~1), 로깅/디버깅 및 msg.confidence용
    if track.is_dynamic:
        track.confidence = min(1.0, track.dynamic_streak / n_confirm)
    else:
        track.confidence = min(1.0, track.static_streak / n_release) if track.hits > 1 else 0.0 #track.hits>1이란 말은 최소 2번 이상 관측된 트랙

    return track.is_dynamic
