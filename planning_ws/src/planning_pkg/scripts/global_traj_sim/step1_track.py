"""Track extraction from a ROS occupancy-grid map (yaml + png).

Produces an ordered, smoothed centerline with track widths, in the same
world frame the ROS stack uses (map yaml origin/resolution).
"""

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.interpolate import splprep, splev
from scipy.spatial import cKDTree

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class Track:
    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        self.resolution = float(meta["resolution"])
        self.origin = np.array(meta["origin"][:2], dtype=float)
        img_path = yaml_path.parent / meta["image"]
        img = np.array(Image.open(img_path).convert("L"), dtype=float)
        if meta.get("negate", 0):
            img = 255.0 - img
        # ROS map_server convention: occupancy p = (255 - pixel) / 255
        occ = (255.0 - img) / 255.0
        self.free_mask = occ < float(meta.get("free_thresh", 0.196))
        self.img_shape = img.shape  # (rows, cols)

    def px_to_world(self, rc):
        """rc: (N,2) array of (row, col) float pixel coords -> world (x, y)."""
        rc = np.atleast_2d(rc)
        x = self.origin[0] + rc[:, 1] * self.resolution
        y = self.origin[1] + (self.img_shape[0] - rc[:, 0]) * self.resolution
        return np.column_stack([x, y])

    def boundaries(self):
        """Return (outer, inner) closed boundary polylines in world coords."""
        # The track is the largest ring-shaped free region: a connected free
        # component that encloses a hole (map background is also "free" but
        # has no hole, so it is rejected).
        labels, n = ndimage.label(self.free_mask)
        sizes = ndimage.sum(self.free_mask, labels, range(1, n + 1))
        ring = None
        for lab in (1 + np.argsort(sizes)[::-1]):
            comp = labels == lab
            # the map background also looks ring-shaped but touches the border
            touches_border = (comp[0, :].any() or comp[-1, :].any()
                              or comp[:, 0].any() or comp[:, -1].any())
            if touches_border:
                continue
            filled = ndimage.binary_fill_holes(comp)
            if filled.sum() - comp.sum() > comp.sum() * 0.05:
                ring = comp
                break
        if ring is None:
            raise RuntimeError("No ring-shaped (closed-loop) free region found in map")
        self.free_mask = ring
        fig = plt.figure()
        cs = plt.contour(self.free_mask.astype(float), levels=[0.5])
        segs = []
        for path in _contour_paths(cs):
            for poly in path.to_polygons(closed_only=False):
                segs.append(np.asarray(poly))
        plt.close(fig)
        # contour() coords are (x=col, y=row); keep the two longest loops
        segs = sorted(segs, key=lambda s: _polyline_length(s), reverse=True)[:2]
        if len(segs) < 2:
            raise RuntimeError("Expected a ring-shaped track (2 boundaries), "
                               f"found {len(segs)} contour(s)")
        polys = []
        for s in segs:
            rc = np.column_stack([s[:, 1], s[:, 0]])  # (row, col)
            polys.append(self.px_to_world(rc))
        # outer loop encloses larger area
        polys.sort(key=lambda p: abs(_signed_area(p)), reverse=True)
        return polys[0], polys[1]


def _contour_paths(cs):
    """matplotlib 3.8+ 는 ContourSet.get_paths(), 그 이전은 cs.collections."""
    if hasattr(cs, "get_paths"):
        return cs.get_paths()
    return [p for coll in cs.collections for p in coll.get_paths()]


def _polyline_length(p):
    return np.sum(np.hypot(*np.diff(p, axis=0).T))


def _signed_area(p):
    x, y = p[:, 0], p[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def _resample_closed(pts, n):
    d = np.hypot(*np.diff(pts, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(d)])
    si = np.linspace(0.0, s[-1], n, endpoint=False)
    x = np.interp(si, s, pts[:, 0])
    y = np.interp(si, s, pts[:, 1])
    return np.column_stack([x, y])


def extract_centerline(track, n_raw=1500, smooth=5.0, ds=0.1):
    """Centerline as midpoints between inner and outer boundary, then a
    periodic smoothing spline resampled at ~ds spacing.

    Returns dict with keys: xy, psi, kappa, s, d_left, d_right (arrays).
    """
    outer, inner = track.boundaries()
    inner_r = _resample_closed(inner, n_raw)
    outer_tree = cKDTree(outer)
    _, idx = outer_tree.query(inner_r)
    mid = 0.5 * (inner_r + outer[idx])

    # periodic smoothing spline
    tck, _ = splprep([mid[:, 0], mid[:, 1]], s=smooth, per=1)
    # dense evaluation to get arc length, then resample uniformly
    u = np.linspace(0, 1, 20000, endpoint=False)
    x, y = splev(u, tck)
    dl = np.hypot(np.diff(x, append=x[0]), np.diff(y, append=y[0]))
    length = dl.sum()
    n_pts = max(int(round(length / ds)), 10)
    s_dense = np.concatenate([[0.0], np.cumsum(dl)])[:-1]
    u_uniform = np.interp(np.linspace(0, length, n_pts, endpoint=False), s_dense, u)

    x, y = splev(u_uniform, tck)
    dx, dy = splev(u_uniform, tck, der=1)
    ddx, ddy = splev(u_uniform, tck, der=2)
    xy = np.column_stack([x, y])
    psi = np.arctan2(dy, dx)
    kappa = (dx * ddy - dy * ddx) / np.power(dx * dx + dy * dy, 1.5)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])

    # widths: distance to each physical boundary
    inner_tree = cKDTree(inner)
    d_inner, i_in = inner_tree.query(xy)
    d_outer, i_out = outer_tree.query(xy)
    # which side is which: sign of cross(tangent, to-boundary)
    t_vec = np.column_stack([dx, dy])
    to_inner = inner[i_in] - xy
    cross = t_vec[:, 0] * to_inner[:, 1] - t_vec[:, 1] * to_inner[:, 0]
    d_left = np.where(cross > 0, d_inner, d_outer)
    d_right = np.where(cross > 0, d_outer, d_inner)

    return {"xy": xy, "psi": psi, "kappa": kappa, "s": s,
            "d_left": d_left, "d_right": d_right,
            "boundaries": (outer, inner)}
