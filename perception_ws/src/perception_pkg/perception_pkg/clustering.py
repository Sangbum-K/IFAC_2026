'''
클러스터링 모듈
- LaserScan 포인트는 이미 각도순으로 정렬되어 있으므로, 인접한 두 포인트 사이
  거리가 임계값 이하면 같은 클러스터로 묶는 breakpoint(연속성) 기반
  클러스터링을 사용한다. 일반 DBSCAN 대비 O(N)으로 훨씬 가볍다.
- 임계값은 고정이 아니라 센서 거리에 비례해 커지는 adaptive breakpoint
  (Borges & Aldon / Forza ETH 계열). 멀수록 포인트 간격이 벌어지는 걸 보정한다.
- **ranges를 points와 따로 받는 이유**: 임계값의 r은 "센서 원점 기준 거리"여야
  한다. 노드는 클러스터링 전에 points를 map frame으로 옮기므로, points만 받아서
  |point|를 쓰면 그건 map 원점 기준 거리가 되어 임계값이 통째로 틀어진다
  (트랙 위 위치에 따라 같은 장애물이 병합되거나 사라진다 — 실측 확인).
'''
import numpy as np

# d_max에 곱하는 보수화 계수. 원식(Borges & Aldon)에는 없고, 이 스택에서
# 과병합보다 과분할을 선호해 절반으로 조인 값이다. 3*sigma 노이즈 항까지
# 같이 줄어들므로 실효 lambda는 설정값보다 다소 작게 동작한다.
SAFETY_FACTOR = 0.5


class Cluster:
    __slots__ = ("points", "centroid", "radius")

    def __init__(self, points):
        self.points = points  # (M, 2)
        self.centroid = points.mean(axis=0)
        # 클러스터 반경: 중심에서 가장 먼 포인트까지 거리
        self.radius = float(np.max(np.linalg.norm(points - self.centroid, axis=1)))


def adaptive_cluster_points(points, ranges, angle_increment,
                            lambda_angle=np.deg2rad(10.0), sigma=0.015,
                            min_cluster_size=5):
    """
    각도순으로 정렬된 포인트를 adaptive breakpoint 기준으로 클러스터링한다.

    인접 포인트 i-1, i를 같은 물체로 볼 최대 거리:

        d_max = (r * sin(dphi) / sin(lambda - dphi) + 3*sigma) * SAFETY_FACTOR

    r은 두 포인트의 센서 거리 평균이다.

    Parameters
    ----------
    points : (N, 2) ndarray
        스캔 순서가 유지된 포인트. 클러스터 기하(centroid/radius)만 여기서
        계산하므로 laser frame이든 map frame이든 무관하다.
    ranges : (N,) array-like
        points와 1:1 대응하는 **센서 원점 기준 거리** [m].
        points를 map frame으로 옮겼더라도 이 값은 laser frame 거리여야 한다.
    angle_increment : float
        LaserScan.angle_increment [rad]
    lambda_angle : float
        허용 표면 입사각 [rad]. 클수록 관대(덜 쪼갬).
    sigma : float
        라이다 거리 노이즈 [m]
    min_cluster_size : int
        노이즈 제거용 최소 포인트 수

    Returns
    -------
    clusters : list[Cluster]
    """
    points = np.asarray(points)
    ranges = np.asarray(ranges, dtype=np.float64)

    if len(points) < max(1, min_cluster_size):
        return []
    if len(ranges) != len(points):
        raise ValueError('ranges must be the same length as points')

    dphi = float(angle_increment)
    # lambda <= dphi면 sin이 0 이하가 되어 d_max가 음수/발산한다. 양수로 클램프해
    # "전부 병합"으로 떨어뜨린다 — 전부 분할(=장애물 실명)보다 안전한 실패 방향이다.
    # 정상 설정에서는 걸리지 않는다 (노드가 기동 시 lambda > dphi를 검증).
    denom = max(float(np.sin(lambda_angle - dphi)), 1e-6)

    # 인접 쌍 단위로 벡터화 — 포인트마다 파이썬 루프를 돌면 1080점 스캔에서
    # 25ms가 걸려 20Hz executor의 TF 콜백을 굶긴다 (실측).
    r_pair = 0.5 * (ranges[:-1] + ranges[1:])
    d_max = (r_pair * np.sin(dphi) / denom + 3.0 * sigma) * SAFETY_FACTOR
    d = np.linalg.norm(points[1:] - points[:-1], axis=1)

    breaks = np.flatnonzero(d > d_max) + 1

    return [Cluster(seg) for seg in np.split(points, breaks)
            if len(seg) >= min_cluster_size]
