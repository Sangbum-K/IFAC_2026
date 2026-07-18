"""
전처리 모듈
- range_min/max 필터링
- FOV(관심 각도 범위) 제한
- median 필터로 스파이크 노이즈 제거
- LaserScan -> (N, 2) xy 포인트 배열로 변환
"""
import numpy as np


def preprocess_scan(ranges, angle_min, angle_increment,
                     range_min=0.05, range_max=10.0,
                     fov_min=-np.pi, fov_max=np.pi,
                     median_kernel=3):
    """
    LaserScan.ranges (list/array) -> 필터링된 (x, y) 포인트 배열 반환

    Parameters
    ----------
    ranges : array-like, scan_msg.ranges
    angle_min : float, scan_msg.angle_min
    angle_increment : float, scan_msg.angle_increment
    range_min/max : 유효 거리 범위 (센서 스펙 + 실사용 범위)
    fov_min/max : 관심 각도 범위 (rad). 차량 전방만 볼 경우 좁게 설정
    median_kernel : 스파이크 노이즈 제거용 median filter 커널 크기 (홀수)

    Returns
    -------
    points : (N, 2) ndarray, [x, y] in laser frame
    """
    ranges = np.asarray(ranges, dtype=np.float32)
    n = len(ranges)
    angles = angle_min + np.arange(n) * angle_increment

    # 1) median filter로 스파이크(순간 튀는 값) 제거
    if median_kernel >= 3 and median_kernel % 2 == 1 and n > median_kernel:
        ranges = _median_filter_1d(ranges, median_kernel)

    # 2) range / FOV 유효성 마스크
    valid = (
        np.isfinite(ranges)
        & (ranges >= range_min)
        & (ranges <= range_max)
        & (angles >= fov_min)
        & (angles <= fov_max)
    )

    r = ranges[valid]
    a = angles[valid]

    xs = r * np.cos(a)
    ys = r * np.sin(a)
    points = np.stack([xs, ys], axis=1)
    return points


def _median_filter_1d(arr, kernel):
    """가장자리는 그대로 두고 내부 구간만 median filter 적용 (간단/저비용 구현)"""
    half = kernel // 2
    out = arr.copy()
    # sliding window median (벡터화된 근사: 작은 커널이므로 반복문 비용 낮음)
    for i in range(half, len(arr) - half):
        window = arr[i - half:i + half + 1]
        finite = window[np.isfinite(window)]
        if len(finite) > 0:
            out[i] = np.median(finite)
    return out
