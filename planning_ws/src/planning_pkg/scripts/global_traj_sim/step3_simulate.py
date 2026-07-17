"""Closed-loop simulation of the global trajectory.

Mirrors the ROS pipeline without ROS:
  - local_trajectory_node: an 80-waypoint local window around the car's s
  - crazy_controller (pure-pursuit mode): speed-scaled L1 lookahead
    L1 = clip(m_l1 * v + q_l1, t_clip_min, t_clip_max)   (pp_params_sim.yaml)
  - plant: kinematic bicycle, wheelbase L = 0.324 m (F1TENTH)
"""

import numpy as np
from scipy.spatial import cKDTree


class PurePursuitSim:
    def __init__(self, traj,
                 wheelbase=0.324, max_steer=0.4189,
                 m_l1=0.3, q_l1=0.4, t_clip_min=0.8, t_clip_max=5.0,
                 a_acc_max=5.5, a_dec_max=7.0,
                 num_local_waypoints=80, dt=0.01):
        self.traj = traj
        self.xy = traj["xy"]
        self.vx_ref = traj["vx"]
        self.s_ref = traj["s"]
        self.n = len(self.xy)
        self.track_len = traj["s"][-1] + np.hypot(*(self.xy[0] - self.xy[-1]))
        self.tree = cKDTree(self.xy)
        self.L = wheelbase
        self.max_steer = max_steer
        self.m_l1, self.q_l1 = m_l1, q_l1
        self.t_clip = (t_clip_min, t_clip_max)
        self.a_acc_max, self.a_dec_max = a_acc_max, a_dec_max
        self.n_local = num_local_waypoints
        self.dt = dt

    def local_window(self, idx):
        return (idx + np.arange(self.n_local)) % self.n

    def lookahead_point(self, pos, idx, l1):
        """First waypoint in the local window at least l1 ahead of the car."""
        win = self.local_window(idx)
        d = np.hypot(*(self.xy[win] - pos).T)
        ahead = np.nonzero(d >= l1)[0]
        j = win[ahead[0]] if len(ahead) else win[-1]
        return self.xy[j], j

    def run(self, n_laps=1, max_time=300.0):
        # start on the trajectory at s=0
        pos = self.xy[0].copy()
        theta = float(self.traj["psi"][0])
        v = 0.5
        idx = 0
        s_travel = 0.0
        prev_s = 0.0
        log = {k: [] for k in ("t", "x", "y", "v", "v_ref", "s", "d_err", "steer")}
        t = 0.0
        laps_done = 0

        while t < max_time:
            # --- frenet_frame_node: closest waypoint ---
            _, idx = self.tree.query(pos)
            s_now = self.s_ref[idx]
            # lap counting via s wrap detection
            if prev_s > 0.8 * self.track_len and s_now < 0.2 * self.track_len:
                laps_done += 1
                if laps_done >= n_laps:
                    break
            prev_s = s_now

            # lateral error (signed, vs. reference line)
            t_hat = np.array([np.cos(self.traj["psi"][idx]),
                              np.sin(self.traj["psi"][idx])])
            e_vec = pos - self.xy[idx]
            d_err = t_hat[0] * e_vec[1] - t_hat[1] * e_vec[0]

            # --- controller ---
            l1 = np.clip(self.m_l1 * v + self.q_l1, *self.t_clip)
            target, j = self.lookahead_point(pos, idx, l1)
            dx, dy = target - pos
            alpha = np.arctan2(dy, dx) - theta
            alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
            ld = max(np.hypot(dx, dy), 1e-3)
            steer = np.clip(np.arctan2(2.0 * self.L * np.sin(alpha), ld),
                            -self.max_steer, self.max_steer)
            v_cmd = self.vx_ref[j]

            # --- plant: kinematic bicycle + accel-limited speed ---
            dv = np.clip(v_cmd - v, -self.a_dec_max * self.dt,
                         self.a_acc_max * self.dt)
            v = max(v + dv, 0.0)
            theta += v / self.L * np.tan(steer) * self.dt
            pos = pos + v * self.dt * np.array([np.cos(theta), np.sin(theta)])
            t += self.dt
            s_travel += v * self.dt

            log["t"].append(t)
            log["x"].append(pos[0]); log["y"].append(pos[1])
            log["v"].append(v); log["v_ref"].append(self.vx_ref[idx])
            log["s"].append(s_now + laps_done * self.track_len)
            log["d_err"].append(d_err); log["steer"].append(steer)

        return {k: np.asarray(v) for k, v in log.items()} | {
            "laps": laps_done, "track_len": self.track_len}
