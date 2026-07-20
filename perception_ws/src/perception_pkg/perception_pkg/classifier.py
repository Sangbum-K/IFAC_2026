"""
Static / dynamic obstacle classification helpers.
"""


def classify_track(track, v_thresh=0.25, n_confirm=5, n_release=8):
    speed = track.kf.speed

    if speed >= v_thresh:
        track.dynamic_streak += 1
        track.static_streak = 0
    else:
        track.dynamic_streak = 0
        track.static_streak = getattr(track, "static_streak", 0) + 1

    if not track.is_dynamic and track.dynamic_streak >= n_confirm:
        track.is_dynamic = True
    elif track.is_dynamic and track.static_streak >= n_release:
        track.is_dynamic = False

    if track.is_dynamic:
        track.confidence = min(1.0, track.dynamic_streak / n_confirm)
    else:
        track.confidence = min(1.0, track.static_streak / n_release) if track.hits > 1 else 0.0

    return track.is_dynamic
