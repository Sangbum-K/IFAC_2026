'''
매칭(Association) + 상태추정(Kalman Filter) 모듈

- 상태벡터: [x, y, vx, vy] (등속도 Constant Velocity 모델)
- 매칭: 이전 프레임 트랙 예측 위치 vs 현재 프레임 클러스터 중심을
  최근접 이웃(Greedy Nearest Neighbor) 방식으로 매칭.
  (장애물 수가 많지 않은 F1TENTH 트랙 특성상 헝가리안 알고리즘 없이도 충분함.
   장애물이 많아지면 scipy.optimize.linear_sum_assignment로 교체 가능)
'''
import numpy as np


class KalmanFilterCV:
    """등속도(Constant Velocity) 모델 2D 칼만필터. 상태: [x, y, vx, vy]"""

    def __init__(self, x0, y0, process_var=0.5, meas_var=0.05):
        self.x = np.array([x0, y0, 0.0, 0.0], dtype=np.float64)  # 상태
        self.P = np.eye(4) * 1.0                                  # 오차 공분산

        self.Q_base = process_var  # 프로세스 노이즈 스케일
        self.R = np.eye(2) * meas_var  # 관측 노이즈 (위치만 관측)

        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float64)

    def predict(self, dt):
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float64)

        # 등속도 모델용 프로세스 노이즈 (표준적인 discretized white noise acceleration)
        q = self.Q_base
        Q = q * np.array([
            [dt**4/4, 0,       dt**3/2, 0],
            [0,       dt**4/4, 0,       dt**3/2],
            [dt**3/2, 0,       dt**2,   0],
            [0,       dt**3/2, 0,       dt**2],
        ], dtype=np.float64)

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z):
        """z: 관측된 [x, y] 위치"""
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
        self.hits = 1          # 총 관측된 횟수
        self.misses = 0        # 연속 미관측 횟수
        self.dynamic_streak = 0  # 연속으로 '동적' 조건을 만족한 프레임 수
        self.is_dynamic = False
        self.confidence = 0.0

    def predict(self, stamp):
        dt = max(stamp - self.last_stamp, 1e-3)
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
    """
    다중 장애물 트래킹 관리자.
    - match(): 현재 프레임 클러스터들과 기존 트랙을 매칭 -> 업데이트 / 신규 트랙 생성 / 소실 트랙 정리
    """

    def __init__(self, gating_distance=0.5, track_timeout=10):
        self.tracks = {}          # id -> Track
        self._next_id = 0
        self.gating_distance = gating_distance  # 매칭 허용 최대 거리 [m]
        self.track_timeout = track_timeout       # 연속 미관측 허용 프레임 수

    def step(self, clusters, stamp):
        """
        한 프레임(스캔) 처리: predict -> associate -> update/spawn -> prune

        Parameters
        ----------
        clusters : list[Cluster], 현재 프레임에서 검출된 클러스터들
        stamp : float, 현재 스캔의 타임스탬프(초)

        Returns
        -------
        active_tracks : list[Track], 현재 활성 상태인 트랙들
        """
        # 1) 모든 트랙 예측 (dt 반영)
        for track in self.tracks.values():
            track.predict(stamp)

        # 2) Greedy Nearest Neighbor 매칭
        unmatched_clusters = list(range(len(clusters)))
        matched_track_ids = set()

        # (트랙, 클러스터) 거리 쌍을 모두 계산 후 가까운 순으로 정렬해 그리디 매칭
        pairs = []
        for tid, track in self.tracks.items():
            tx, ty = track.kf.position
            for ci in unmatched_clusters:
                cx, cy = clusters[ci].centroid
                d = float(np.hypot(tx - cx, ty - cy))
                if d <= self.gating_distance:
                    pairs.append((d, tid, ci))
        pairs.sort(key=lambda p: p[0])

        used_clusters = set()
        for d, tid, ci in pairs:
            if tid in matched_track_ids or ci in used_clusters:
                continue
            self.tracks[tid].update(clusters[ci], stamp)
            matched_track_ids.add(tid)
            used_clusters.add(ci)

        # 3) 매칭 안 된 트랙: miss 처리
        for tid, track in self.tracks.items():
            if tid not in matched_track_ids:
                track.mark_missed()

        # 4) 매칭 안 된 클러스터: 신규 트랙 생성
        for ci in range(len(clusters)):
            if ci not in used_clusters:
                self._spawn(clusters[ci], stamp)

        # 5) 오래 미관측된 트랙 제거
        stale_ids = [tid for tid, t in self.tracks.items() if t.misses > self.track_timeout]
        for tid in stale_ids:
            del self.tracks[tid]

        return list(self.tracks.values())

    def _spawn(self, cluster, stamp):
        track = Track(self._next_id, cluster, stamp)
        self.tracks[self._next_id] = track
        self._next_id += 1
