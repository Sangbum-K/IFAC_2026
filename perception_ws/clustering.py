'''
클러스터링 모듈
- LaserScan 포인트는 이미 각도순으로 정렬되어 있으므로,
  인접한 두 포인트 사이 거리가 임계값(eps) 이하면 같은 클러스터로 묶는
  breakpoint(연속성) 기반 클러스터링을 사용한다.
- 일반 DBSCAN 대비 O(N)으로 훨씬 가볍고, 정렬된 laser scan에 최적화되어 있다.
'''
import numpy as np


class Cluster:
    __slots__ = ("points", "centroid", "radius")

    def __init__(self, points):
        self.points = points  # (M, 2)
        self.centroid = points.mean(axis=0)
        # 클러스터 반경: 중심에서 가장 먼 포인트까지 거리
        self.radius = float(np.max(np.linalg.norm(points - self.centroid, axis=1)))


def cluster_points(points, eps=0.18, min_cluster_size=3):
    """
    각도순으로 정렬된 (N, 2) 포인트 배열을 인접 거리 기준으로 클러스터링한다.

    Parameters
    ----------
    points : (N, 2) ndarray, preprocess_scan()의 출력 (각도순 정렬 유지 가정)
    eps : 같은 물체로 볼 최대 인접 포인트 간 거리 [m]
    min_cluster_size : 노이즈 제거용 최소 포인트 수

    Returns
    -------
    clusters : list[Cluster]
    """
    if len(points) == 0:
        return []

    clusters = []
    current = [points[0]]

    for i in range(1, len(points)):
        d = np.linalg.norm(points[i] - points[i - 1])
        if d <= eps:
            current.append(points[i])
        else:
            _flush(current, clusters, min_cluster_size)
            current = [points[i]]
    _flush(current, clusters, min_cluster_size)

    # LaserScan은 360도 원형이므로 시작/끝 클러스터가 실제로는 이어진 경우가 있음.
    # 필요 시(FOV가 360도 전체일 때) 아래 병합 로직을 활성화한다.
    if len(clusters) >= 2:
        first, last = clusters[0], clusters[-1]
        if np.linalg.norm(first.points[0] - last.points[-1]) <= eps:
            merged_points = np.concatenate([last.points, first.points], axis=0)
            clusters[0] = Cluster(merged_points)
            clusters.pop()

    return clusters


def _flush(point_list, clusters, min_cluster_size):
    if len(point_list) >= min_cluster_size:
        clusters.append(Cluster(np.array(point_list)))
