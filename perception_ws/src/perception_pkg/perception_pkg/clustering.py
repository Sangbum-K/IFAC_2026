"""
Breakpoint clustering for angle-ordered LaserScan points.
"""
import numpy as np


class Cluster:
    __slots__ = ("points", "centroid", "radius")

    def __init__(self, points):
        self.points = points
        self.centroid = points.mean(axis=0)
        self.radius = float(np.max(np.linalg.norm(points - self.centroid, axis=1)))


def cluster_points(points, eps=0.18, min_cluster_size=3):
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
