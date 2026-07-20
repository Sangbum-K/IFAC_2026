"""
Greedy nearest-neighbor tracking with a constant-velocity Kalman filter.
"""
import numpy as np


class KalmanFilterCV:
    def __init__(self, x0, y0, process_var=0.5, meas_var=0.05):
        self.x = np.array([x0, y0, 0.0, 0.0], dtype=np.float64)
        self.P = np.eye(4) * 1.0
        self.Q_base = process_var
        self.R = np.eye(2) * meas_var
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float64)

    def predict(self, dt):
        F = np.array(
            [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]],
            dtype=np.float64,
        )
        q = self.Q_base
        Q = q * np.array(
            [
                [dt**4 / 4, 0, dt**3 / 2, 0],
                [0, dt**4 / 4, 0, dt**3 / 2],
                [dt**3 / 2, 0, dt**2, 0],
                [0, dt**3 / 2, 0, dt**2],
            ],
            dtype=np.float64,
        )
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    @property
    def position(self):
        return self.x[0], self.x[1]

    @property
    def velocity(self):
        return self.x[2], self.x[3]

    @property
    def speed(self):
        return float(np.hypot(self.x[2], self.x[3]))


class Track:
    def __init__(self, track_id, cluster, stamp):
        self.id = track_id
        self.kf = KalmanFilterCV(cluster.centroid[0], cluster.centroid[1])
        self.last_stamp = stamp
        self.radius = cluster.radius
        self.hits = 1
        self.misses = 0
        self.dynamic_streak = 0
        self.static_streak = 0
        self.is_dynamic = False
        self.confidence = 0.0

    def predict(self, stamp):
        dt = stamp - self.last_stamp
        if dt <= 0.0:
            dt = 1e-3
        elif dt > 0.5:
            dt = 0.5
        self.kf.predict(dt)
        self.last_stamp = stamp

    def update(self, cluster, stamp):
        self.kf.update(cluster.centroid)
        self.radius = cluster.radius
        self.hits += 1
        self.misses = 0
        self.last_stamp = stamp

    def mark_missed(self):
        self.misses += 1


class Tracker:
    def __init__(self, gating_distance=0.5, track_timeout=10):
        self.tracks = {}
        self._next_id = 0
        self.gating_distance = gating_distance
        self.track_timeout = track_timeout

    def step(self, clusters, stamp):
        for track in self.tracks.values():
            track.predict(stamp)

        matched_track_ids = set()
        pairs = []
        for tid, track in self.tracks.items():
            tx, ty = track.kf.position
            for ci, cluster in enumerate(clusters):
                cx, cy = cluster.centroid
                d = float(np.hypot(tx - cx, ty - cy))
                if d <= self.gating_distance:
                    pairs.append((d, tid, ci))
        pairs.sort(key=lambda p: p[0])

        used_clusters = set()
        for _, tid, ci in pairs:
            if tid in matched_track_ids or ci in used_clusters:
                continue
            self.tracks[tid].update(clusters[ci], stamp)
            matched_track_ids.add(tid)
            used_clusters.add(ci)

        for tid, track in self.tracks.items():
            if tid not in matched_track_ids:
                track.mark_missed()

        for ci, cluster in enumerate(clusters):
            if ci not in used_clusters:
                self._spawn(cluster, stamp)

        stale_ids = [tid for tid, t in self.tracks.items() if t.misses > self.track_timeout]
        for tid in stale_ids:
            del self.tracks[tid]

        return list(self.tracks.values())

    def _spawn(self, cluster, stamp):
        track = Track(self._next_id, cluster, stamp)
        self.tracks[self._next_id] = track
        self._next_id += 1
