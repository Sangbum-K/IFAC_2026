'''
매칭(Association) + 상태추정(Kalman Filter) + 재식별(Re-ID) 모듈

- 상태벡터: [x, y, vx, vy] (등속도 Constant Velocity 모델), map 프레임.
- 매칭: 이전 프레임 트랙 예측 위치 vs 현재 프레임 클러스터 중심을
  최근접 이웃(Greedy Nearest Neighbor) 방식으로 매칭.
  (장애물 수가 많지 않은 F1TENTH 트랙 특성상 헝가리안 없이도 충분.
   많아지면 scipy.optimize.linear_sum_assignment로 교체 가능)
- 재식별(Re-ID): timeout으로 소멸한 트랙을 잠깐 보관했다가, 곧 근처에서
  다시 나타난 클러스터에 같은 id를 부활시킨다. H2H에선 상대가 딱 1대라
  occlusion(코너 뒤, 자차 사각) 후에도 같은 id를 유지해야 GP 예측 버퍼와
  상대 정체성(ever_dynamic)이 끊기지 않는다.
- 위치 이력(pos_hist): 정적/동적 판별의 "위치 분산" 근거 (classifier.py).
'''
from collections import deque

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
    def __init__(self, track_id, cluster, stamp, pos_hist_len=20):
        self.id = track_id
        self.kf = KalmanFilterCV(cluster.centroid[0], cluster.centroid[1])
        self.last_stamp = stamp
        self.radius = cluster.radius
        self.hits = 1          # 총 관측된 횟수
        self.misses = 0        # 연속 미관측 횟수
        self.dynamic_streak = 0  # 연속으로 '동적' 조건을 만족한 프레임 수
        self.static_streak = 0   # 연속으로 '정적' 조건을 만족한 프레임 수
        self.is_dynamic = False
        # 한 번이라도 동적 확정된 적이 있나 = "상대 차량" 정체성. 이후 정차/코너
        # 감속으로 잠깐 느려져도 이 플래그로 dynamic을 오래 유지한다(classifier).
        self.ever_dynamic = False
        self.confidence = 0.0
        # map 프레임 "측정" 위치 이력 (정적/동적 위치분산 판별용).
        # 측정된 프레임만 append — 코스팅(외삽) 위치는 분산을 왜곡하므로 넣지 않는다.
        self.pos_hist = deque(maxlen=pos_hist_len)
        self.pos_hist.append((float(cluster.centroid[0]), float(cluster.centroid[1])))

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
        self.pos_hist.append((float(cluster.centroid[0]), float(cluster.centroid[1])))

    def mark_missed(self):
        self.misses += 1


class Tracker:
    """
    다중 장애물 트래킹 관리자.
    - step(): predict -> associate -> update/spawn(+re-ID) -> prune(-> retire)
    """

    def __init__(self, gating_distance=0.5, track_timeout=10,
                 reid_distance=0.7, reid_timeout=1.0, pos_hist_len=20,
                 radius_ratio_max=0.0):
        self.tracks = {}          # id -> Track
        self._next_id = 0
        self.gating_distance = gating_distance  # 매칭 허용 최대 거리 [m]
        self.track_timeout = track_timeout       # 연속 미관측 허용 프레임 수
        self.reid_distance = reid_distance       # 재식별 허용 거리 [m]
        self.reid_timeout = reid_timeout         # 소멸 트랙 보관 시간 [s]
        self.pos_hist_len = pos_hist_len
        # 크기 일관성 게이트: 트랙이 보던 반경과 후보 클러스터 반경의 비가
        # 이 배수를 넘으면 같은 물체로 보지 않는다. 0이면 비활성.
        self.radius_ratio_max = radius_ratio_max
        self.retired = []         # 최근 소멸 트랙 (재식별 후보): list[dict]

    def step(self, clusters, stamp):
        """
        한 프레임(스캔) 처리: predict -> associate -> update/spawn(+re-ID) -> prune

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
                if d > self.gating_distance:
                    continue
                # ── 크기 일관성 게이트 ────────────────────────────────────
                # 거리만 보면 근처의 작은 노이즈 클러스터가 진짜 물체보다
                # 가깝다는 이유로 트랙 ID를 가져간다. rosbag_0806 run_234734
                # #53 실측: 반경 0.157m 실물(2.05,0.13)과 반경 0.005m 파편
                # (2.35,0.10) 사이를 같은 ID가 왕복해 KF 속도가 0.47 m/s로
                # 튀었고 정지 물체가 "동적"으로 분류됐다 (전체 검출의
                # 30%가 동적, 자차 정지 중에도 centroid 산포 p90 0.16~0.55m).
                # 같은 물체라면 반경이 프레임 간 몇 배씩 변하지 않는다.
                if self.radius_ratio_max > 1.0:
                    r_t = max(float(track.radius), 1e-3)
                    r_c = max(float(clusters[ci].radius), 1e-3)
                    if max(r_t / r_c, r_c / r_t) > self.radius_ratio_max:
                        continue
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

        # 4) 매칭 안 된 클러스터: 재식별 시도 후 신규 트랙 생성
        for ci in range(len(clusters)):
            if ci not in used_clusters:
                self._spawn(clusters[ci], stamp)

        # 5) 오래 미관측된 트랙 제거 -> 재식별용으로 잠깐 보관(retire)
        stale_ids = [tid for tid, t in self.tracks.items()
                     if t.misses > self.track_timeout]
        for tid in stale_ids:
            self._retire(self.tracks[tid], stamp)
            del self.tracks[tid]

        # 6) 보관 기한 지난 재식별 후보 폐기
        self.retired = [r for r in self.retired
                        if stamp - r['t'] <= self.reid_timeout]

        return list(self.tracks.values())

    def _retire(self, track, stamp):
        x, y = track.kf.position
        vx, vy = track.kf.velocity
        self.retired.append({
            'id': track.id, 'x': x, 'y': y, 'vx': vx, 'vy': vy, 't': stamp,
            'ever_dynamic': track.ever_dynamic, 'is_dynamic': track.is_dynamic,
        })

    def _spawn(self, cluster, stamp):
        cx = float(cluster.centroid[0])
        cy = float(cluster.centroid[1])

        # 재식별: 최근 소멸 트랙을 등속 외삽한 예측 위치가 이 클러스터와
        # reid_distance 이내이면 같은 id를 부활 (정체성 유지). 자차 사각/코너
        # 뒤 occlusion 후 재등장하는 상대가 새 id를 받지 않게 한다.
        best_k, best_d = None, self.reid_distance
        for k, r in enumerate(self.retired):
            dt = stamp - r['t']
            px = r['x'] + r['vx'] * dt
            py = r['y'] + r['vy'] * dt
            d = float(np.hypot(px - cx, py - cy))
            if d <= best_d:
                best_d, best_k = d, k
        if best_k is not None:
            r = self.retired.pop(best_k)
            track = Track(r['id'], cluster, stamp, self.pos_hist_len)
            track.ever_dynamic = r['ever_dynamic']
            track.is_dynamic = r['is_dynamic']  # 재등장 즉시 정체성 복원
            self.tracks[r['id']] = track
            return

        track = Track(self._next_id, cluster, stamp, self.pos_hist_len)
        self.tracks[self._next_id] = track
        self._next_id += 1
