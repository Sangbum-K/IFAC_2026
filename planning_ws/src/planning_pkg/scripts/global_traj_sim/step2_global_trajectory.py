"""Global trajectory generation: IQP raceline + velocity profile + CSV export.

두 종류의 raceline을 만들 수 있다:
  1. centerline  — step1의 중앙선을 그대로 사용 (경로 최적화 없음)
  2. IQP raceline — TUM 스타일 최소곡률(iterative QP) 최적화.
     centerline의 각 점을 법선 방향으로 alpha만큼 이동시키되
     (p_race = p_center + alpha * n), 트랙 폭 제약 안에서 곡률 제곱합을
     최소화한다. 코너 진입 전 아웃, 에이펙스 인, 탈출 아웃의 레이싱
     라인이 자동으로 나온다.

Output CSV is byte-compatible with planner_pkg/global_trajectory_node.cpp:
9 semicolon-separated columns
  s_m; x_m; y_m; d_right; d_left; psi_rad; kappa_radpm; vx_mps; ax_mps2
"""

import numpy as np
from scipy.interpolate import splprep, splev
from scipy.optimize import lsq_linear
from scipy.spatial import cKDTree

G = 9.81


# ============================================================ IQP raceline
def _respline_closed(xy, ds, smooth=0.5):
    """폐곡선 점열 -> 주기 스플라인 스무딩 + 균일 ds 리샘플.

    반환: dict(s, xy, psi, kappa). step1의 스플라인 처리와 같은 방식.
    IQP 반복마다 점 간격이 흐트러지므로(법선 이동의 부작용) 매 반복
    후 균일 간격으로 되돌려야 이산 곡률 근사가 유효하게 유지된다.
    """
    tck, _ = splprep([xy[:, 0], xy[:, 1]], s=smooth, per=1)
    u = np.linspace(0, 1, 8 * len(xy), endpoint=False)
    x, y = splev(u, tck)
    dl = np.hypot(np.diff(x, append=x[0]), np.diff(y, append=y[0]))
    length = dl.sum()
    n_pts = max(int(round(length / ds)), 16)
    s_dense = np.concatenate([[0.0], np.cumsum(dl)])[:-1]
    u_uni = np.interp(np.linspace(0, length, n_pts, endpoint=False), s_dense, u)
    x, y = splev(u_uni, tck)
    dx, dy = splev(u_uni, tck, der=1)
    ddx, ddy = splev(u_uni, tck, der=2)
    psi = np.arctan2(dy, dx)
    kappa = (dx * ddy - dy * ddx) / np.power(dx * dx + dy * dy, 1.5)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    return {"s": s, "xy": np.column_stack([x, y]), "psi": psi, "kappa": kappa}


def _compute_widths(xy, psi, outer, inner, long_window=1.0, k=32):
    """경로 각 점에서 좌/우 벽까지 거리 (step1과 동일한 부호 규약: +d=왼쪽).

    단순 최근접(유클리드) 벽점은 헤어핀에서 코너 건너편 벽을 잡거나
    좌/우 부호가 뒤집히는 글리치를 만든다 (예: 이웃 0.65m인데 한 점만 0.05m).
    -> 후보 벽점 중 접선 방향 거리 |long| < long_window 인 '옆에 있는' 점만
    사용하고, 부호 있는 횡거리(lat)로 좌/우를 직접 판별한다.
    """
    walls = np.vstack([outer, inner])
    tree = cKDTree(walls)
    k = min(k, len(walls))
    _, idx = tree.query(xy, k=k)                       # (N, k)
    rel = walls[idx] - xy[:, None, :]                  # (N, k, 2)
    t_vec = np.column_stack([np.cos(psi), np.sin(psi)])
    lon = rel[:, :, 0] * t_vec[:, None, 0] + rel[:, :, 1] * t_vec[:, None, 1]
    lat = -rel[:, :, 0] * t_vec[:, None, 1] + rel[:, :, 1] * t_vec[:, None, 0]

    beside = np.abs(lon) < long_window
    left_cand = np.where(beside & (lat > 0.0), lat, np.inf)
    right_cand = np.where(beside & (lat < 0.0), -lat, np.inf)
    d_left = left_cand.min(axis=1)
    d_right = right_cand.min(axis=1)

    # 후보가 없던 점(k개 이웃이 전부 반대편 등)은 이웃 값으로 보간
    for d in (d_left, d_right):
        bad = ~np.isfinite(d)
        if bad.any():
            good_idx = np.flatnonzero(~bad)
            d[bad] = np.interp(np.flatnonzero(bad), good_idx, d[good_idx],
                               period=len(d))
    return d_left, d_right


def optimize_raceline_iqp(center, boundaries, safe_dist_m=0.30,
                          n_iters=3, ds_opt=0.5, ds_out=0.1, objective="curvature"):
    """TUM 스타일 raceline 최적화 (최소곡률 IQP 또는 최단거리 SP).

    수식 (UNICORN/TUM global_racetrajectory_optimization과 동일 구조):
      objective="curvature" (IQP, 메인 주행 라인):
        min_alpha  sum ||D2 (c + alpha*n)||^2   (D2 = 주기 2차 차분 = 곡률 근사)
      objective="length" (SP, 추월 라인 — UNICORN lane_change_planner가 쓰는 것):
        min_alpha  sum ||D1 (c + alpha*n)||^2   (D1 = 주기 1차 차분 = 구간 길이)
      s.t.       -(d_right - safe) <= alpha <= (d_left - safe)
    목적이 alpha에 비선형(경로 갱신 시 법선/폭이 변함)이므로
    "풀고 -> 경로 갱신 -> 다시 풀기"를 n_iters회 반복한다(Iterative QP).
    각 QP는 경계 있는 최소제곱이라 scipy.optimize.lsq_linear로 푼다.

    파라미터:
      safe_dist_m — 벽에서 확보할 거리 = 차 반폭 + 안전여유.
                    UNICORN의 safety_width에 해당. 줄이면 벽에 붙어 빨라지고
                    (위험), 키우면 안전하지만 랩타임 손해.
      n_iters     — IQP 반복 수. 2~3이면 수렴 (매 반복 max|alpha| 감소 확인).
      ds_opt      — 최적화 그리드 간격. 좁히면 정밀/느림 (TUM 기본도 ~1m급).
    반환: extract_centerline과 같은 키의 dict (s, xy, psi, kappa, d_left, d_right).
    """
    outer, inner = boundaries
    tag = "IQP" if objective == "curvature" else "SP"
    # 최적화용 성긴 그리드로 다운샘플 (n~700이면 dense lsq도 수 초 내)
    stride = max(int(round(ds_opt / (center["s"][1] - center["s"][0]))), 1)
    ref = {k: center[k][::stride].copy()
           for k in ("xy", "psi", "d_left", "d_right")}

    for it in range(n_iters):
        n = len(ref["xy"])
        normal = np.column_stack([-np.sin(ref["psi"]), np.cos(ref["psi"])])
        if objective == "curvature":
            # 주기 2차 차분 행렬 D2 (균일 간격 가정 -> 매 반복 리샘플로 보장)
            D = (np.roll(np.eye(n), -1, axis=1) - 2 * np.eye(n)
                 + np.roll(np.eye(n), 1, axis=1))
        else:
            # 주기 1차 차분 행렬 D1: 구간 길이 제곱 합 최소화 = 최단거리
            D = np.roll(np.eye(n), -1, axis=1) - np.eye(n)
        # ||A alpha - b||^2 꼴로 전개: x/y 성분을 세로로 쌓음
        A = np.vstack([D @ np.diag(normal[:, 0]), D @ np.diag(normal[:, 1])])
        b = -np.concatenate([D @ ref["xy"][:, 0], D @ ref["xy"][:, 1]])
        # 바운드: 현재 기준선이 이미 safe_dist를 위반한 점(스무딩이 apex를
        # 깎은 경우)은 바운드가 음수가 되어 QP가 강제로 되밀어낸다.
        # (기존 max(...,0) 클램프는 위반을 "더 나빠지지만 않게" 방치했음)
        lo = -(ref["d_right"] - safe_dist_m)
        hi = ref["d_left"] - safe_dist_m
        narrow = lo > hi          # 복도 폭 < 2*safe 인 구간은 가운데로
        mid = 0.5 * (lo + hi)
        lo = np.where(narrow, mid - 1e-6, lo)
        hi = np.where(narrow, mid + 1e-6, hi)
        res = lsq_linear(A, b, bounds=(lo, hi))
        alpha = res.x

        # 경로 갱신 -> 리샘플 -> 폭 재계산 (다음 반복의 선형화 기준점)
        xy_new = ref["xy"] + alpha[:, None] * normal
        geo = _respline_closed(xy_new, ds_opt)
        d_left, d_right = _compute_widths(geo["xy"], geo["psi"], outer, inner)
        ref = {"xy": geo["xy"], "psi": geo["psi"],
               "d_left": d_left, "d_right": d_right}
        print(f"      {tag} iter {it + 1}/{n_iters}: max|alpha|={np.max(np.abs(alpha)):.3f} m, "
              f"max|kappa|={np.max(np.abs(geo['kappa'])):.3f} 1/m")

    # 최종 정리 QP: 루프 마지막의 스무딩이 남긴 위반을 한 번 더 복구
    # (바운드가 음수화되어 위반점을 강제로 되밀어낸다. 목적함수는 동일)
    n = len(ref["xy"])
    normal = np.column_stack([-np.sin(ref["psi"]), np.cos(ref["psi"])])
    if objective == "curvature":
        D = (np.roll(np.eye(n), -1, axis=1) - 2 * np.eye(n)
             + np.roll(np.eye(n), 1, axis=1))
    else:
        D = np.roll(np.eye(n), -1, axis=1) - np.eye(n)
    A = np.vstack([D @ np.diag(normal[:, 0]), D @ np.diag(normal[:, 1])])
    b = -np.concatenate([D @ ref["xy"][:, 0], D @ ref["xy"][:, 1]])
    lo = -(ref["d_right"] - safe_dist_m)
    hi = ref["d_left"] - safe_dist_m
    narrow = lo > hi
    mid = 0.5 * (lo + hi)
    lo = np.where(narrow, mid - 1e-6, lo)
    hi = np.where(narrow, mid + 1e-6, hi)
    alpha = lsq_linear(A, b, bounds=(lo, hi)).x
    xy_final = ref["xy"] + alpha[:, None] * normal

    # 출력 해상도(ds_out) 리샘플은 스무딩 없이(보간) — 제약 만족점을 보존.
    # 절점 사이 처짐은 ds_opt=0.5m 간격에서 cm 수준이라 무시 가능.
    geo = _respline_closed(xy_final, ds_out, smooth=0.0)
    d_left, d_right = _compute_widths(geo["xy"], geo["psi"], outer, inner)
    min_clear = min(d_left.min(), d_right.min())
    if min_clear < safe_dist_m - 0.1:
        print(f"      [warn] 최종 벽 여유 {min_clear:.2f} m < safe_dist {safe_dist_m:.2f} m "
              f"(좁은 구간이거나 보정 한계)")
    return {"s": geo["s"], "xy": geo["xy"], "psi": geo["psi"],
            "kappa": geo["kappa"], "d_left": d_left, "d_right": d_right}


def velocity_profile(s, kappa, v_max, mu, a_acc_max, a_dec_max, n_iters=3):
    """Forward-backward pass over a closed track (friction-circle limited)."""
    kappa = np.abs(kappa)
    n = len(s)
    ds = np.diff(s, append=s[-1] + (s[-1] - s[-2]))
    a_lat_max = mu * G
    with np.errstate(divide="ignore"):
        v = np.minimum(np.sqrt(a_lat_max / np.maximum(kappa, 1e-9)), v_max)

    for _ in range(n_iters):  # iterate so the closed-loop wrap converges
        # forward pass (acceleration limit)
        for i in range(n):
            j = (i + 1) % n
            a_lat = v[i] ** 2 * kappa[i]
            a_avail = a_acc_max * np.sqrt(max(0.0, 1.0 - (a_lat / a_lat_max) ** 2))
            v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2.0 * a_avail * ds[i]))
        # backward pass (braking limit)
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            a_lat = v[j] ** 2 * kappa[j]
            a_avail = a_dec_max * np.sqrt(max(0.0, 1.0 - (a_lat / a_lat_max) ** 2))
            v[i] = min(v[i], np.sqrt(v[j] ** 2 + 2.0 * a_avail * ds[i]))
    return v


def build_trajectory(center, v_max=7.0, mu=0.85, a_acc_max=5.5, a_dec_max=7.0,
                     margin=0.15):
    s, kappa = center["s"], center["kappa"]
    v = velocity_profile(s, kappa, v_max, mu, a_acc_max, a_dec_max)
    ds = np.diff(s, append=s[-1] + (s[-1] - s[-2]))
    v_next = np.roll(v, -1)
    ax = (v_next ** 2 - v ** 2) / (2.0 * ds)
    return {
        "s": s,
        "xy": center["xy"],
        "d_right": np.maximum(center["d_right"] - margin, 0.0),
        "d_left": np.maximum(center["d_left"] - margin, 0.0),
        "psi": center["psi"],
        "kappa": kappa,
        "vx": v,
        "ax": ax,
    }


def roll_to_start(traj, start_xy=(0.0, 0.0), forward_x=True):
    """Rotate waypoint order so s=0 is nearest start_xy, and flip loop
    direction so the tangent there points toward +x (the sim start pose)."""
    xy = traj["xy"]
    i0 = int(np.argmin(np.hypot(xy[:, 0] - start_xy[0], xy[:, 1] - start_xy[1])))
    flip = forward_x and np.cos(traj["psi"][i0]) < 0.0

    def reorder(a):
        a = np.roll(a, -i0, axis=0)
        if flip:  # keep start point first, traverse the loop the other way
            a = np.concatenate([a[:1], a[1:][::-1]])
        return a.copy()

    out = {k: reorder(v) for k, v in traj.items() if k != "s"}
    if flip:
        out["psi"] = np.arctan2(np.sin(out["psi"] + np.pi), np.cos(out["psi"] + np.pi))
        out["kappa"] = -out["kappa"]
        out["d_left"], out["d_right"] = out["d_right"], out["d_left"]
    # rebuild s from geometry
    xy = out["xy"]
    seg = np.hypot(*np.diff(xy, axis=0).T)
    out["s"] = np.concatenate([[0.0], np.cumsum(seg)])
    return out


def write_csv(traj, path, header_note=""):
    cols = np.column_stack([
        traj["s"], traj["xy"][:, 0], traj["xy"][:, 1],
        traj["d_right"], traj["d_left"], traj["psi"],
        traj["kappa"], traj["vx"], traj["ax"],
    ])
    with open(path, "w") as f:
        if header_note:
            f.write(f"# {header_note}\n")
        f.write("# s_m; x_m; y_m; d_right; d_left; psi_rad; kappa_radpm; vx_mps; ax_mps2\n")
        for row in cols:
            f.write("; ".join(f"{v:.7f}" for v in row) + "\n")
