"""
기존 버전과의 차이점
--------------------------------
기존 preprocess_scan()은 laser frame 좌표에 boundary_polygon을 바로 적용했습니다.
문제는 트랙 경계(boundary_polygon)는 원래 "고정된 지형"이라 map frame에서 정의하는 게
자연스러운데, laser frame은 차량이 움직일 때마다 계속 바뀌는 프레임이라서
"laser frame 기준 고정 폴리곤"이라는 개념 자체가 성립하지 않습니다.
(차량이 움직이면 매 프레임 폴리곤을 다시 계산해서 넣어줘야 하는데, 그 변환 로직이
 없어서 실제로는 차량이 원점에 있다고 가정한 채 동작하는 것과 같았습니다.)

그래서 이번 버전은:
  1) preprocess_scan()은 순수하게 "센서 프레임 기준 필터링 + xy 변환"만 담당 (boundary_polygon 파라미터 제거)
  2) transform_points()로 laser frame -> map(또는 원하는 target) frame 변환을 별도로 수행
  3) map frame으로 옮겨진 포인트에 대해 filter_points_in_polygon()으로 경계 필터를 적용
이렇게 책임을 나누면 boundary_polygon은 map frame에 딱 한 번만 정의하면 되고,
차량이 어디에 있든 항상 올바르게 동작합니다.
"""
import numpy as np


def preprocess_scan(ranges, angle_min, angle_increment,
                     range_min=0.05, range_max=10.0,
                     fov_min=-np.pi, fov_max=np.pi,
                     median_kernel=3):
    """
    LaserScan.ranges (list/array) -> 필터링된 (x, y) 포인트 배열 반환 (laser frame 기준)

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
    point_ranges : (N,) ndarray, 각 포인트의 센서 원점 기준 거리 [m]
        adaptive 클러스터링의 임계값이 이 값을 쓴다. points는 이후 map frame으로
        옮겨지므로 |point|로는 복원할 수 없어 따로 돌려준다.
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

    return points, r.astype(np.float64)


def transform_points(points, translation, rotation_matrix_2d):
    """
    points를 (translation, rotation_matrix_2d)로 정의된 변환에 따라
    다른 좌표계로 옮긴다. (예: laser frame -> map frame)

    Parameters
    ----------
    points : (N, 2) ndarray, 변환 전 좌표
    translation : (2,) array-like, [tx, ty]
    rotation_matrix_2d : (2, 2) ndarray, 2D 회전행렬

    Returns
    -------
    (N, 2) ndarray, 변환된 좌표
    """
    if len(points) == 0:
        return points
    t = np.asarray(translation, dtype=np.float64).reshape(1, 2)
    R = np.asarray(rotation_matrix_2d, dtype=np.float64)
    return points @ R.T + t


def rotation_matrix_2d_from_quaternion(qx, qy, qz, qw):
    """
    쿼터니언에서 2D(z축 회전만) 회전행렬을 추출.
    라이다/차량은 보통 평면 위에서만 회전하므로 yaw만 뽑아 쓰면 충분하다.
    """
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def polygon_mask(points, polygon):
    """
    polygon(원하는 frame, 보통 map frame 기준) 내부에 있는 포인트의 bool 마스크.

    points와 나란히 관리되는 부수 배열(예: 센서 거리)을 같은 마스크로 잘라야
    하므로, 필터 결과가 아니라 마스크를 돌려주는 형태를 기본으로 둔다.

    Parameters
    ----------
    points : (N, 2) ndarray, polygon과 동일한 frame 기준 좌표여야 함
    polygon : (M, 2) ndarray 또는 list, M>=3

    Returns
    -------
    (N,) bool ndarray. 폴리곤이 없거나 부적합하면 전부 True (필터 미적용).
    """
    if polygon is None or len(points) == 0:
        return np.ones(len(points), dtype=bool)
    polygon = np.asarray(polygon, dtype=np.float32)
    if polygon.ndim != 2 or polygon.shape[0] < 3 or polygon.shape[1] != 2:
        return np.ones(len(points), dtype=bool)
    return _points_in_polygon(points, polygon)


def filter_points_in_polygon(points, polygon):
    """polygon 내부 포인트만 남긴다 (polygon_mask의 편의 래퍼)."""
    return points[polygon_mask(points, polygon)]


def _median_filter_1d(arr, kernel):
    """가장자리는 그대로 두고 내부 구간만 median filter 적용.

    윈도 안의 비유한값(inf = 미반사, nan)은 제외하고 중앙값을 낸다.
    윈도가 전부 비유한값이면 원본을 유지한다.

    포인트마다 파이썬 루프를 돌면 1080빔 스캔에서 53ms가 걸려 on_scan 전체
    (~58ms)를 지배한다 — process_rate_hz 20을 못 채우고 실효 15Hz로 떨어지며,
    싱글스레드 executor가 TF 콜백을 굶는 원인이 된다 (bag 재생 실측).
    sliding_window_view로 벡터화해 결과는 동일하게 두고 비용만 없앤다.
    """
    half = kernel // 2
    out = arr.copy()
    if half < 1 or len(arr) <= 2 * half:
        return out

    win = np.lib.stride_tricks.sliding_window_view(arr, kernel)
    finite = np.isfinite(win)
    # 전부 비유한값인 윈도는 nanmedian이 nan + RuntimeWarning을 내므로 제외하고,
    # 해당 위치는 out에 그대로 둔다 (기존 동작).
    valid = finite.any(axis=1)
    if valid.any():
        masked = np.where(finite[valid], win[valid], np.nan)
        out[half:len(arr) - half][valid] = np.nanmedian(masked, axis=1)
    return out


def _points_in_polygon(points, polygon):
    """Ray casting 기반 point-in-polygon 테스트."""
    x = points[:, 0]
    y = points[:, 1]
    inside = np.zeros(len(points), dtype=bool)
    x1 = polygon[:, 0]
    y1 = polygon[:, 1]
    x2 = np.roll(x1, -1)
    y2 = np.roll(y1, -1)
    for ax, ay, bx, by in zip(x1, y1, x2, y2):
        intersects = ((ay > y) != (by > y))
        denom = (by - ay) + 1e-12
        x_intersection = (bx - ax) * (y - ay) / denom + ax
        inside ^= intersects & (x < x_intersection)
    return inside
