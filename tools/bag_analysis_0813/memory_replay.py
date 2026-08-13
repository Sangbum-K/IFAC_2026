#!/usr/bin/env python3
"""랩 기억 레지스트리를 run_0813_165312 실측 스트림으로 재생 검증한다.

bridge 노드의 memory_* 로직만 떼어내 그대로 실행한다 (ROS 불필요).
확인 항목:
  1. S/F 콘이 몇 랩 만에 확정되는가, 그 뒤 몇 m 앞에서 주입되는가
  2. 접촉 후 동적 오분류(7건)가 기억을 파괴하지 않는가  <- poison 거리 게이트
  3. 우벽 밖 파편(d -0.53~-0.82)이 확정되지 않는가 / carving 으로 지워지는가
  4. 확정된 기억이 라이브를 덮지 않는가
"""
import math
import pickle
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

D = pickle.load(open(Path(__file__).parent / "data.pkl", "rb"))
TL = 32.07


class Registry:
    """perception_obstacle_bridge_node 의 memory_* 를 그대로 옮긴 것."""

    def __init__(self, confirm_laps=2, match_s=0.60, match_d=0.35,
                 inject_ahead=12.0, dyn_suppress=1.5, forget_misses=2,
                 poison_min_dist=1.0, relevant_margin=0.25, half_width=0.15,
                 close_requires_match=3.0, track_length=TL):
        self.confirm_laps = confirm_laps
        self.match_s = match_s
        self.match_d = match_d
        self.inject_ahead = inject_ahead
        self.dyn_suppress = dyn_suppress
        self.forget_misses = forget_misses
        self.poison_min_dist = poison_min_dist
        self.relevant_margin = relevant_margin
        self.close_requires_match = close_requires_match
        self.half_width = half_width
        self.track_length = track_length
        self.memory = []
        self.next_id = 900000
        self.lap_index = 0
        self.log = []

    def ds(self, a, b):
        return ((a - b + self.track_length / 2.0) % self.track_length
                - self.track_length / 2.0)

    def find(self, s, d):
        best, best_cost = None, float('inf')
        for a in self.memory:
            ds = abs(self.ds(s, a['s']))
            dd = abs(d - a['d'])
            if ds > self.match_s or dd > self.match_d:
                continue
            cost = (ds / self.match_s) ** 2 + (dd / self.match_d) ** 2
            if cost < best_cost:
                best, best_cost = a, cost
        return best

    def observe(self, s, d, half, fwd, is_dynamic, measured):
        far_dynamic = is_dynamic and fwd >= self.poison_min_dist
        a = self.find(s, d)
        if a is None:
            if far_dynamic or not measured or is_dynamic:
                return
            if abs(d) - half - self.half_width > self.relevant_margin:
                return
            a = dict(s=s, d=d, half=half, laps=set(), confirmed=False,
                     misses=0, poisoned=False, best_dist=float('inf'),
                     best_s=s, best_d=d, best_half=half,
                     mem_id=self.next_id, matched_this_pass=False,
                     pass_open=False)
            self.next_id += 1
            self.memory.append(a)
        if far_dynamic:
            if not a['poisoned']:
                self.log.append(
                    f"  lap{self.lap_index} POISON s={a['s']:.2f} d={a['d']:+.2f} "
                    f"(fwd {fwd:.2f}m 에서 동적)")
            a['poisoned'] = True
            return
        if is_dynamic or not measured:
            if measured:
                a['matched_this_pass'] = True
            return
        a['matched_this_pass'] = True
        if 0.0 <= fwd < a['best_dist']:
            a['best_dist'] = fwd
            a['best_s'], a['best_d'], a['best_half'] = s, d, half
        a['laps'].add(self.lap_index)
        if not a['confirmed'] and len(a['laps']) >= self.confirm_laps:
            a['confirmed'] = True
            self.log.append(
                f"  lap{self.lap_index} CONFIRM s={a['s']:.2f} d={a['d']:+.2f} "
                f"half={a['half']:.2f} (랩 {sorted(a['laps'])})")

    def lap_boundary(self):
        for a in self.memory:
            if math.isfinite(a['best_dist']):
                w = 0.5 if a['confirmed'] else 1.0
                a['s'] = (a['s'] + w * self.ds(a['best_s'], a['s'])) \
                    % self.track_length
                a['d'] += w * (a['best_d'] - a['d'])
                a['half'] += w * (a['best_half'] - a['half'])
                a['misses'] = 0
            elif a['pass_open']:
                a['misses'] += 1
            a['pass_open'] = False
            a['matched_this_pass'] = False
            a['best_dist'] = float('inf')
        self.merge()
        keep = []
        for a in self.memory:
            if a['poisoned'] or a['misses'] >= self.forget_misses:
                self.log.append(
                    f"  lap{self.lap_index} DROP s={a['s']:.2f} d={a['d']:+.2f} "
                    f"(misses={a['misses']} poisoned={a['poisoned']})")
                continue
            keep.append(a)
        self.memory = keep
        self.lap_index += 1

    def merge(self):
        merged = True
        while merged and len(self.memory) > 1:
            merged = False
            for i in range(len(self.memory)):
                for j in range(i + 1, len(self.memory)):
                    a, b = self.memory[i], self.memory[j]
                    if abs(self.ds(a['s'], b['s'])) > self.match_s \
                            or abs(a['d'] - b['d']) > self.match_d:
                        continue
                    keep, drop = (a, b) if len(a['laps']) >= len(b['laps']) else (b, a)
                    keep['laps'] |= drop['laps']
                    keep['poisoned'] = keep['poisoned'] or drop['poisoned']
                    keep['misses'] = min(keep['misses'], drop['misses'])
                    keep['half'] = max(keep['half'], drop['half'])
                    if not keep['confirmed'] and len(keep['laps']) >= self.confirm_laps:
                        keep['confirmed'] = True
                    self.memory.remove(drop)
                    merged = True
                    break
                if merged:
                    break

    def inject(self, ego_s, live):
        """live = [(s, d, is_dynamic)]. 주입된 앵커 목록을 돌려준다."""
        out = []
        for a in self.memory:
            if a['poisoned']:
                continue
            ahead = (a['s'] - ego_s) % self.track_length
            if ahead > self.inject_ahead:
                continue
            a['pass_open'] = True
            if not a['confirmed']:
                continue
            if ahead < self.close_requires_match and not a['matched_this_pass']:
                continue
            if any(abs(self.ds(a['s'], e[0])) <= self.match_s and
                   abs(a['d'] - e[1]) <= self.match_d for e in live):
                continue
            if any(e[2] and abs(self.ds(a['s'], e[0])) <= self.dyn_suppress
                   for e in live):
                continue
            out.append((a, ahead))
        return out


def main():
    fre = D["/car_state/frenet/odom"]
    reg = Registry()

    def ego(t):
        i = np.searchsorted(fre[:, 0], t)
        i = min(max(i, 0), len(fre) - 1)
        return fre[i, 1], fre[i, 2]

    last_s = None
    first_inject = {}      # mem_id -> (t, ahead)
    inject_stats = []
    frames = 0
    for t, obs in D["obs_frenet"]:
        s_e, d_e = ego(t)
        if last_s is not None and last_s - s_e > TL * 0.5:
            reg.lap_boundary()
        last_s = s_e
        frames += 1

        live = []
        for o in obs:
            live.append((o["s_center"], o["d_center"], not o["is_static"]))
        # 주입은 관측 반영 뒤 (라이브 우선). 순서는 노드와 동일.
        for o in obs:
            half = max(o["size"] / 2.0, 0.05)
            fwd = reg.ds(o["s_center"], s_e)
            reg.observe(o["s_center"], o["d_center"], half, fwd,
                        not o["is_static"], bool(o["is_visible"]))
        for a, ahead in reg.inject(s_e, live):
            if a["mem_id"] not in first_inject:
                first_inject[a["mem_id"]] = (t, ahead)
            inject_stats.append((t, a["mem_id"], ahead, a["s"], a["d"]))

    print(f"프레임 {frames}, 랩 {reg.lap_index}")
    print("\n=== 레지스트리 이벤트 ===")
    for line in reg.log:
        print(line)

    print(f"\n=== 최종 기억 {len(reg.memory)}개 ===")
    for a in reg.memory:
        print(f"  s={a['s']:6.2f} d={a['d']:+.3f} half={a['half']:.3f} "
              f"confirmed={a['confirmed']} laps={len(a['laps'])} "
              f"misses={a['misses']}")

    print(f"\n=== 주입 통계 (총 {len(inject_stats)} 프레임) ===")
    if inject_stats:
        by = {}
        for t, mid, ahead, s, d in inject_stats:
            by.setdefault(mid, []).append(ahead)
        for mid, aheads in by.items():
            a = np.array(aheads)
            t0, ahead0 = first_inject[mid]
            print(f"  mem{mid}: {len(a)}프레임, 최초 주입 t={t0:.2f} "
                  f"거리 {ahead0:.2f}m, 주입 거리 p50 {np.median(a):.2f} "
                  f"max {a.max():.2f}m")
    else:
        print("  (주입 없음)")

    # 반사실: 확정된 기억이 있을 때 S/F 접촉 시점에 몇 m 앞에서 알았을까
    print("\n=== S/F 접촉 5건에서 기억이 준 선행거리 ===")
    contacts = [124.41, 157.29, 166.51, 175.86, 213.88]
    for tc in contacts:
        s_e, _ = ego(tc)
        best = None
        for t, mid, ahead, s, d in inject_stats:
            if t > tc:
                break
            if abs(((s - s_e + TL / 2) % TL) - TL / 2) < 1.5:
                if best is None or t > best[0]:
                    best = (t, ahead)
        # 그 접촉 대상 기억이 "처음 주입된" 시각 = 그 랩 접근 시작
        lead = None
        for t, mid, ahead, s, d in inject_stats:
            if tc - 4.0 <= t <= tc and abs(((s - s_e + TL / 2) % TL) - TL / 2) < 2.5:
                if lead is None or ahead > lead[1]:
                    lead = (t, ahead)
        if lead:
            print(f"  t={tc:.2f}: 최대 선행 인지거리 {lead[1]:.2f} m "
                  f"(t={lead[0]:.2f}) — 실측 첫 탐지는 5.7~5.8 m")
        else:
            print(f"  t={tc:.2f}: 주입 없음")


if __name__ == "__main__":
    main()
