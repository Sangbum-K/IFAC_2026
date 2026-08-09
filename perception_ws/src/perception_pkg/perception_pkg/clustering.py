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
    __slots__ = (
        "points", "centroid", "radius", "beam_indices",
        "source_component", "source_radius", "source_size",
    )

    def __init__(self, points, beam_indices=None, source_component=-1,
                 source_radius=None, source_size=None):
        self.points = points  # (M, 2)
        self.centroid = points.mean(axis=0)
        # 클러스터 반경: 중심에서 가장 먼 포인트까지 거리
        self.radius = float(np.max(np.linalg.norm(points - self.centroid, axis=1)))
        self.beam_indices = (None if beam_indices is None
                             else np.asarray(beam_indices, dtype=np.int64))
        self.source_component = int(source_component)
        self.source_radius = (self.radius if source_radius is None
                              else float(source_radius))
        self.source_size = (len(points) if source_size is None
                            else int(source_size))


def _adaptive_breaks(points, ranges, angle_increment, lambda_angle, sigma,
                     beam_indices=None, source_labels=None):
    """Return split positions while respecting removed-beam angular gaps."""
    count = len(points)
    if count <= 1:
        return np.empty(0, dtype=np.int64)
    if beam_indices is None:
        beam_indices = np.arange(count, dtype=np.int64)
    else:
        beam_indices = np.asarray(beam_indices, dtype=np.int64)
    if len(beam_indices) != count or np.any(np.diff(beam_indices) <= 0):
        raise ValueError('beam_indices must be strictly increasing and align with points')

    dphi = np.abs(np.diff(beam_indices)) * abs(float(angle_increment))
    valid_geometry = dphi < float(lambda_angle)
    denom = np.sin(float(lambda_angle) - dphi)
    r_pair = 0.5 * (ranges[:-1] + ranges[1:])
    d_max = np.full(count - 1, -np.inf, dtype=np.float64)
    d_max[valid_geometry] = (
        r_pair[valid_geometry] * np.sin(dphi[valid_geometry])
        / np.maximum(denom[valid_geometry], 1e-9)
        + 3.0 * sigma
    ) * SAFETY_FACTOR
    distance = np.linalg.norm(points[1:] - points[:-1], axis=1)
    split = distance > d_max
    if source_labels is not None:
        labels = np.asarray(source_labels, dtype=np.int64)
        if len(labels) != count:
            raise ValueError('source_labels must align with points')
        split |= labels[1:] != labels[:-1]
    return np.flatnonzero(split) + 1


def adaptive_component_provenance(points, ranges, angle_increment,
                                  lambda_angle=np.deg2rad(10.0), sigma=0.015,
                                  beam_indices=None):
    """Label raw scan components before map/track filters can fragment them."""
    points = np.asarray(points, dtype=np.float64)
    ranges = np.asarray(ranges, dtype=np.float64)
    if len(points) != len(ranges):
        raise ValueError('ranges must be the same length as points')
    if beam_indices is None:
        beam_indices = np.arange(len(points), dtype=np.int64)
    else:
        beam_indices = np.asarray(beam_indices, dtype=np.int64)
    if not len(points):
        return (np.empty(0, dtype=np.int64), np.empty(0, dtype=np.float64),
                np.empty(0, dtype=np.int64))
    breaks = _adaptive_breaks(
        points, ranges, angle_increment, lambda_angle, sigma, beam_indices)
    labels = np.empty(len(points), dtype=np.int64)
    radii = []
    sizes = []
    starts = np.r_[0, breaks]
    ends = np.r_[breaks, len(points)]
    for label, (start, end) in enumerate(zip(starts, ends)):
        component = points[start:end]
        centroid = component.mean(axis=0)
        radius = float(np.max(np.linalg.norm(component - centroid, axis=1)))
        labels[start:end] = label
        radii.append(radius)
        sizes.append(end - start)
    return labels, np.asarray(radii), np.asarray(sizes, dtype=np.int64)


def adaptive_cluster_points(points, ranges, angle_increment,
                            lambda_angle=np.deg2rad(10.0), sigma=0.015,
                            min_cluster_size=5, beam_indices=None,
                            source_labels=None, source_radii=None,
                            source_sizes=None):
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

    if beam_indices is None:
        beam_indices = np.arange(len(points), dtype=np.int64)
    else:
        beam_indices = np.asarray(beam_indices, dtype=np.int64)
    labels = (None if source_labels is None
              else np.asarray(source_labels, dtype=np.int64))
    breaks = _adaptive_breaks(
        points, ranges, angle_increment, lambda_angle, sigma,
        beam_indices, labels)
    starts = np.r_[0, breaks]
    ends = np.r_[breaks, len(points)]
    clusters = []
    for start, end in zip(starts, ends):
        if end - start < min_cluster_size:
            continue
        source = -1 if labels is None else int(labels[start])
        source_radius = None
        source_size = None
        if source >= 0:
            if source_radii is not None:
                source_radius = np.asarray(source_radii)[source]
            if source_sizes is not None:
                source_size = np.asarray(source_sizes)[source]
        clusters.append(Cluster(
            points[start:end], beam_indices[start:end], source,
            source_radius, source_size))
    return clusters
