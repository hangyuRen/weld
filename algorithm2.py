import sys
import math
import re
from dataclasses import dataclass

import numpy as np
import pyvista as pv


# 替代 pcl::PointXYZ
@dataclass
class PointXYZ:
    x: float = math.nan
    y: float = math.nan
    z: float = math.nan


# 替代 FeatureResult 结构体
@dataclass
class FeatureResult:
    ts: str = ""
    ok: bool = False

    criticalL_idx: int = -1
    criticalR_idx: int = -1
    base_idx: int = -1

    criticalL: PointXYZ = None
    criticalR: PointXYZ = None
    base: PointXYZ = None

    z_target: float = math.nan

    leftTarget_idx: int = -1
    rightTarget_idx: int = -1
    leftTarget: PointXYZ = None
    rightTarget: PointXYZ = None

    msg: str = ""

    def __post_init__(self):
        if self.criticalL is None: self.criticalL = PointXYZ()
        if self.criticalR is None: self.criticalR = PointXYZ()
        if self.base is None: self.base = PointXYZ()
        if self.leftTarget is None: self.leftTarget = PointXYZ()
        if self.rightTarget is None: self.rightTarget = PointXYZ()


# 去除 {}[]()" 空白等冗余字符并分割
def split_csv4(line):
    # 过滤掉冗余字符
    clean_line = re.sub(r'[{}[\]()"]', '', line)
    # 分割并去除前后空白
    tokens = [t.strip() for t in clean_line.split(',')]
    tokens = [t for t in tokens if t]

    if len(tokens) < 4:
        return False, "", 0.0, 0.0, 0.0

    ts = tokens[0]
    try:
        x = float(tokens[1])
        y = float(tokens[2])
        z = float(tokens[3])
        return True, ts, x, y, z
    except ValueError:
        return False, "", 0.0, 0.0, 0.0


# 点到线段距离（3D）
def point_to_segment_distance(p0, p1, p2):
    vx, vy, vz = p2.x - p1.x, p2.y - p1.y, p2.z - p1.z
    wx, wy, wz = p0.x - p1.x, p0.y - p1.y, p0.z - p1.z

    vv = vx * vx + vy * vy + vz * vz
    if vv < 1e-12:
        # p1 与 p2 几乎重合
        return math.sqrt(wx * wx + wy * wy + wz * wz)

    t = (wx * vx + wy * vy + wz * vz) / vv
    t = max(0.0, min(1.0, t))

    qx = p1.x + t * vx
    qy = p1.y + t * vy
    qz = p1.z + t * vz

    dx, dy, dz = p0.x - qx, p0.y - qy, p0.z - qz

    return math.sqrt(dx * dx + dy * dy + dz * dz)


# RDP：返回保留点的原始索引（升序，包含首尾）
def rdp_indices(pts, s, e, eps):
    if e <= s:
        return [s]
    if e == s + 1:
        return [s, e]

    maxDist = -1.0
    idx = -1
    for i in range(s + 1, e):
        d = point_to_segment_distance(pts[i], pts[s], pts[e])
        if d > maxDist:
            maxDist = d
            idx = i

    if maxDist > eps and idx >= 0:
        left = rdp_indices(pts, s, idx, eps)
        right = rdp_indices(pts, idx, e, eps)
        # 合并，避免 idx 重复
        left.pop()
        left.extend(right)
        return left
    else:
        return [s, e]


def process_scanline(ts, linePts):
    """Detect the two corners of a weld groove.

    The weld cross-section has the shape of a peak (in z): two flat regions
    connected by a rising slope on the left and a falling slope on the right.
    The two corners are the transition points between flat and steep.
    The algorithm uses slope analysis to find these transitions precisely.
    """
    r = FeatureResult(ts=ts)
    n = len(linePts)

    if n < 30:
        r.msg = "scanline too short (<30 points)"
        return r

    # extract z and y as numpy arrays
    z_arr = np.array([p.z for p in linePts])
    y_arr = np.array([p.y for p in linePts])

    # 0) denoise: median filter to remove isolated z spikes
    #    a point is replaced by local median if it deviates too much
    dw = 5  # half-window for median filter
    for i in range(dw, n - dw):
        local = z_arr[i - dw:i + dw + 1]
        med = np.median(local)
        dev = abs(z_arr[i] - med)
        # only replace if deviation is much larger than local noise
        local_mad = np.median(np.abs(local - med)) * 1.4826
        noise = max(local_mad, 0.01)
        if dev > 5.0 * noise:
            z_arr[i] = med

    # 1) smooth z to reduce noise
    win = max(5, n // 50)
    if win % 2 == 0:
        win += 1
    kernel = np.ones(win) / win
    z_s = np.convolve(z_arr, kernel, mode='same')

    # 2) compute slope dz/dy using a window
    sw = max(3, n // 60)
    slope = np.zeros(n)
    for i in range(sw, n - sw):
        dy = y_arr[i + sw] - y_arr[i - sw]
        dz = z_s[i + sw] - z_s[i - sw]
        if abs(dy) > 1e-9:
            slope[i] = dz / dy

    abs_slope = np.abs(slope)

    # 3) determine baseline noise level and active threshold
    valid = abs_slope[sw:n - sw]
    if len(valid) == 0:
        r.msg = "not enough valid slope points"
        return r

    baseline = np.median(valid)
    threshold = max(baseline * 4.0, 0.08)

    # 4) find contiguous active regions where |slope| > threshold
    regions = []
    in_region = False
    start = 0
    for i in range(sw, n - sw):
        if abs_slope[i] > threshold and not in_region:
            start = i
            in_region = True
        elif abs_slope[i] <= threshold and in_region:
            regions.append((start, i))
            in_region = False
    if in_region:
        regions.append((start, n - sw))

    if len(regions) < 2:
        r.msg = f"found {len(regions)} active region(s), need >= 2"
        return r

    # 5) merge nearby regions with same sign (gap < 3*win)
    #    do NOT merge regions with opposite slope sign: they are
    #    the two edges of the groove separated by a flat top
    merged = [list(regions[0])]
    for s, e in regions[1:]:
        prev_s, prev_e = merged[-1]
        dy_prev = y_arr[prev_e] - y_arr[prev_s]
        dz_prev = z_s[prev_e] - z_s[prev_s]
        slope_prev = dz_prev / dy_prev if abs(dy_prev) > 1e-9 else 0.0

        dy_cur = y_arr[e] - y_arr[s]
        dz_cur = z_s[e] - z_s[s]
        slope_cur = dz_cur / dy_cur if abs(dy_cur) > 1e-9 else 0.0

        gap = s - prev_e
        same_sign = (slope_prev * slope_cur) > 0

        if gap < 3 * win and same_sign:
            merged[-1][1] = e
        else:
            merged.append([s, e])

    # 6) filter regions by width and amplitude
    candidates = []
    min_width = max(3, win // 3)
    for s, e in merged:
        width = e - s
        if width < min_width:
            continue
        z_amp = abs(z_s[e] - z_s[s])
        if z_amp < 0.15:
            continue
        candidates.append((s, e, z_amp))

    if len(candidates) < 2:
        r.msg = f"found {len(candidates)} valid groove edge(s), need >= 2"
        return r

    # 7) find the best pair: one rising edge (slope > 0) + one falling edge
    # the pair should be adjacent with a flat top between them
    best_pair = None
    best_score = -1.0

    for i in range(len(candidates) - 1):
        s1, e1, amp1 = candidates[i]
        s2, e2, amp2 = candidates[i + 1]

        # gap between two edges
        gap = s2 - e1

        # slopes
        dy1 = y_arr[e1] - y_arr[s1]
        dz1 = z_s[e1] - z_s[s1]
        slope1 = dz1 / dy1 if abs(dy1) > 1e-9 else 0.0

        dy2 = y_arr[e2] - y_arr[s2]
        dz2 = z_s[e2] - z_s[s2]
        slope2 = dz2 / dy2 if abs(dy2) > 1e-9 else 0.0

        # want opposite-sign slopes (one rising, one falling)
        if slope1 * slope2 >= 0:
            continue

        # the region between e1 and s2 should be relatively flat (top of groove)
        if gap > 5:
            mid_z = z_s[e1:s2]
            mid_var = np.std(mid_z) if len(mid_z) > 1 else 0
            if mid_var > 0.3:
                continue

        # score: prefer large amplitude and moderate gap
        score = (amp1 + amp2) * (1.0 / (1.0 + abs(gap - 10) / 50.0))

        if score > best_score:
            best_score = score
            best_pair = (s1, e1, s2, e2)

    if best_pair is None:
        r.msg = "no valid rising+falling edge pair found"
        return r

    s1, e1, s2, e2 = best_pair

    # 8) refine corner positions to the exact transition point
    # The weld profile (left to right) is:
    #   left-flat -> left-slope -> top-flat -> right-slope -> right-flat
    # Left corner  = boundary between left-flat and left-slope  (near s1)
    # Right corner = boundary between right-slope and right-flat (near e2)

    # walk from s1 backward into the flat region to find the precise boundary
    cL_idx = _refine_corner(abs_slope, s1, -1, sw, threshold * 0.5)
    # walk from e2 forward into the flat region to find the precise boundary
    cR_idx = _refine_corner(abs_slope, e2, 1, sw, threshold * 0.5)

    # ensure cL < cR
    if cL_idx >= cR_idx:
        r.msg = "corner order inverted, skipping"
        return r

    # 9) assign results
    r.criticalL_idx = cL_idx
    r.criticalR_idx = cR_idx
    r.criticalL = linePts[cL_idx]
    r.criticalR = linePts[cR_idx]
    r.base_idx = (cL_idx + cR_idx) // 2
    r.base = linePts[r.base_idx]

    r.leftTarget = r.criticalL
    r.rightTarget = r.criticalR
    r.leftTarget_idx = cL_idx
    r.rightTarget_idx = cR_idx

    r.ok = True
    r.msg = "ok"
    return r


def _refine_corner(abs_slope, anchor, direction, sw, threshold):
    """Walk from anchor in the given direction to find the precise
    transition point between flat and steep regions.

    direction: -1 = walk backward (toward smaller indices)
               +1 = walk forward (toward larger indices)
    Returns the index of the last point that is still 'steep' (above
    threshold), i.e. the boundary of the transition.
    """
    n = len(abs_slope)
    last_steep = anchor
    step = 0
    while True:
        i = anchor + direction * step
        if i < sw or i >= n - sw:
            break
        if abs_slope[i] > threshold:
            last_steep = i
        elif abs_slope[i] < threshold * 0.5:
            break
        step += 1
        if step > 50:
            break
    return last_steep


def _mad_filter(points, sigma=3.5):
    """Remove gross outliers from weld points using MAD (Median
    Absolute Deviation) on y and z separately. Returns cleaned
    points sorted by x.

    Parameters
    ----------
    points : array-like, shape (N, 3)
        Raw detected weld points (x, y, z).
    sigma : float
        Points beyond sigma * MAD-scale are removed.

    Returns
    -------
    np.ndarray, shape (M, 3)  where M <= N
    """
    pts = np.array(points, dtype=float)
    pts = pts[np.argsort(pts[:, 0])]

    for col in (1, 2):
        vals = pts[:, col]
        med = np.median(vals)
        mad = np.median(np.abs(vals - med))
        scale = 1.4826 * mad if mad > 1e-9 else np.std(vals)
        if scale > 1e-9:
            mask = np.abs(vals - med) < sigma * scale
            if mask.sum() >= 5:
                pts = pts[mask]

    return pts


def _local_outlier_filter(points, k=7, sigma=5.0, n_iter=1):
    """Remove points that deviate from the local weld trajectory.

    For each point, fits a local line (y~x, z~x) on its k nearest
    neighbors (by x) and computes the 3D residual. Points whose
    residual exceeds median + sigma * MAD are removed. Iterates
    n_iter times.

    This catches points that pass global MAD but deviate locally
    (e.g. z-spike at weld end, y-jitter mid-section).

    Parameters
    ----------
    points : array-like, shape (N, 3)
        Weld points (x, y, z), already MAD-filtered and sorted by x.
    k : int
        Number of nearest neighbors for local fit.
    sigma : float
        Removal threshold in units of MAD above median residual.
    n_iter : int
        Number of iterations.

    Returns
    -------
    np.ndarray, shape (M, 3)  where M <= N
    """
    pts = np.array(points, dtype=float)
    pts = pts[np.argsort(pts[:, 0])]

    for _ in range(n_iter):
        n = len(pts)
        if n < k + 2:
            break

        xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]
        residuals = np.empty(n)

        for i in range(n):
            # k nearest neighbors by x (including self)
            dx = np.abs(xs - xs[i])
            nbr = np.argsort(dx)[:k]
            nbr_x, nbr_y, nbr_z = xs[nbr], ys[nbr], zs[nbr]

            # local linear fit
            A = np.vstack([nbr_x, np.ones(len(nbr_x))]).T
            ay, by = np.linalg.lstsq(A, nbr_y, rcond=None)[0]
            az, bz = np.linalg.lstsq(A, nbr_z, rcond=None)[0]

            ry = ys[i] - (ay * xs[i] + by)
            rz = zs[i] - (az * xs[i] + bz)
            residuals[i] = np.sqrt(ry ** 2 + rz ** 2)

        med_r = np.median(residuals)
        mad_r = np.median(np.abs(residuals - med_r)) * 1.4826
        if mad_r < 1e-9:
            break

        thresh = med_r + sigma * mad_r
        mask = residuals <= thresh

        if mask.all() or mask.sum() < k + 2:
            break
        pts = pts[mask]

    return pts


def fit_and_resample(points, n_samples=20, degree=3, n_iter=3,
                     sigma_thresh=2.5, x_range=None):
    """Fit a 3D semi-circle to weld points and resample evenly along
    the arc.

    The weld trajectory lies on a pipe surface, so the x-z projection
    is a circular arc. The algorithm:

      1. MAD pre-filter on y and z to remove gross outliers.
      2. Fit a circle in the x-z plane via least squares.
      3. Determine the angular extent of the arc from the data.
      4. Sample n_samples points evenly along the arc angle (first
         and last samples are the arc endpoints).
      5. For y, interpolate linearly along x on the cleaned data.

    Parameters
    ----------
    points : array-like, shape (N, 3)
        Raw detected weld points (x, y, z).
    n_samples : int
        Number of points to sample on the fitted arc (including both
        endpoints).
    x_range : tuple (x_min, x_max) or None
        Unused, kept for API compatibility.

    Returns
    -------
    np.ndarray, shape (n_samples, 3)
        Resampled points on the fitted arc.
    """
    pts = np.array(points, dtype=float)
    pts = pts[np.argsort(pts[:, 0])]

    # --- MAD pre-filter on y and z ---
    for col in (1, 2):
        vals = pts[:, col]
        med = np.median(vals)
        mad = np.median(np.abs(vals - med))
        scale = 1.4826 * mad if mad > 1e-9 else np.std(vals)
        if scale > 1e-9:
            mask = np.abs(vals - med) < 3.5 * scale
            if mask.sum() >= 5:
                pts = pts[mask]

    xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]
    n = len(pts)

    # --- iterative circle fit with outlier removal ---
    # Fit circle, remove points with high radial residual, refit.
    # This removes points that pass global MAD but deviate from the
    # circular trajectory (e.g. y-jitter, z-spike at weld ends).
    for _ in range(3):
        A = np.column_stack([2 * xs, 2 * zs, np.ones(n)])
        b = xs ** 2 + zs ** 2
        sol = np.linalg.lstsq(A, b, rcond=None)[0]
        cx, cz = sol[0], sol[1]
        R = np.sqrt(sol[2] + cx ** 2 + cz ** 2)

        dists = np.sqrt((xs - cx) ** 2 + (zs - cz) ** 2)
        circ_resid = np.abs(dists - R)

        med_r = np.median(circ_resid)
        mad_r = np.median(np.abs(circ_resid - med_r)) * 1.4826
        if mad_r < 1e-9:
            break

        thresh = med_r + 3.0 * mad_r
        mask = circ_resid <= thresh
        if mask.all() or mask.sum() < 10:
            break
        xs, ys, zs = xs[mask], ys[mask], zs[mask]
        n = len(xs)

    # --- determine angular extent ---
    angles = np.arctan2(zs - cz, xs - cx)
    # The arc spans < 180 deg. Use the most extreme angle in the
    # first/last few points (sorted by x) as endpoints so the
    # sampled arc reaches the actual data boundaries.
    # Using median here would shrink the arc inward, missing the
    # true start/end points.
    n_edge = min(5, n // 4)
    if np.median(angles[:n_edge]) <= np.median(angles[-n_edge:]):
        # angles increasing from start to end
        ang_start = np.min(angles[:n_edge])   # at smallest x
        ang_end = np.max(angles[-n_edge:])    # at largest x
    else:
        # angles decreasing from start to end
        ang_start = np.max(angles[:n_edge])
        ang_end = np.min(angles[-n_edge:])

    # --- sample n_samples along arc ---
    angles_sample = np.linspace(ang_start, ang_end, n_samples)
    x_arc = cx + R * np.cos(angles_sample)
    z_arc = cz + R * np.sin(angles_sample)

    # --- y: linear interpolation along x ---
    # x_arc may not be perfectly monotonic near arc edges, so sort
    # both data and query for stable interpolation.
    y_arc = np.interp(x_arc, xs, ys)

    return np.column_stack([x_arc, y_arc, z_arc])


def _smooth_edge_aware(arr, w):
    """Moving average with proper edge handling (window shrinks at
    boundaries so no zero-padding artefacts)."""
    n = len(arr)
    result = np.empty(n)
    for i in range(n):
        lo = max(0, i - w)
        hi = min(n, i + w + 1)
        result[i] = np.mean(arr[lo:hi])
    return result


def _interp_with_regression_extrap(x_data, y_data, x_query, n_end=10):
    """Linear interpolation within data range; linear-regression
    extrapolation outside (using the nearest n_end points).

    Extrapolation slope is clamped to the median absolute per-step
    change inside the data, keeping extrapolation conservative on
    curved weld trajectories where the slope accelerates near ends.
    """
    result = np.interp(x_query, x_data, y_data)

    # median absolute per-step change inside data (for clamping)
    steps = np.abs(np.diff(y_data))
    med_step = np.median(steps) if len(steps) > 0 else 0.0

    below = x_query < x_data[0]
    if below.any():
        n_use = min(n_end, len(x_data))
        xr = x_data[:n_use]
        yr = y_data[:n_use]
        A = np.vstack([xr, np.ones(len(xr))]).T
        a, b = np.linalg.lstsq(A, yr, rcond=None)[0]
        # clamp slope to median step (conservative extrapolation)
        max_slope = med_step
        slope = np.clip(a, -max_slope, max_slope)
        result[below] = slope * x_query[below] + (yr[0] - slope * xr[0])

    above = x_query > x_data[-1]
    if above.any():
        n_use = min(n_end, len(x_data))
        xr = x_data[-n_use:]
        yr = y_data[-n_use:]
        A = np.vstack([xr, np.ones(len(xr))]).T
        a, b = np.linalg.lstsq(A, yr, rcond=None)[0]
        max_slope = med_step
        slope = np.clip(a, -max_slope, max_slope)
        result[above] = slope * x_query[above] + (yr[-1] - slope * xr[-1])

    return result


def main():
    if len(sys.argv) < 2:
        print("Usage: python algorithm2.py <input.txt> [n_samples=20]")
        sys.exit(1)

    inputPath = sys.argv[1]
    n_samples = int(sys.argv[2]) if len(sys.argv) >= 3 else 20

    all_points = []
    left_raw = []
    right_raw = []
    stats = {"ok": 0, "fail": 0}

    try:
        with open(inputPath, 'r', encoding='utf-8') as ifs:

            cur_ts = ""
            cur_line = []

            def flush_group(ts, pts):
                if not ts or not pts:
                    return
                r = process_scanline(ts, pts)
                if r.ok:
                    stats["ok"] += 1
                    left_raw.append([r.criticalL.x, r.criticalL.y, r.criticalL.z])
                    right_raw.append([r.criticalR.x, r.criticalR.y, r.criticalR.z])
                else:
                    stats["fail"] += 1

            # read line by line, group by timestamp
            for line in ifs:
                success, ts, x, y, z = split_csv4(line)
                if not success:
                    continue

                all_points.append([x, y, z])

                if not cur_ts:
                    cur_ts = ts

                if ts != cur_ts:
                    flush_group(cur_ts, cur_line)
                    cur_ts = ts
                    cur_line = []

                cur_line.append(PointXYZ(x, y, z))

            flush_group(cur_ts, cur_line)

    except FileNotFoundError as e:
        print(f"Failed to open file: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Scanlines: {stats['ok']} ok, {stats['fail']} failed, "
          f"{len(left_raw)} raw weld points")

    # post-process: MAD filter + circle-arc fit + resample, per side
    weld_points = []
    if len(left_raw) >= 5:
        left_clean = _mad_filter(left_raw)
        left_fit = fit_and_resample(left_clean, n_samples=n_samples)
        weld_points.extend(left_fit.tolist())
        print(f"Left:  {len(left_raw)} raw -> {len(left_clean)} MAD -> "
              f"{len(left_fit)} arc samples  "
              f"x=[{left_fit[0,0]:.1f}, {left_fit[-1,0]:.1f}]")
    if len(right_raw) >= 5:
        right_clean = _mad_filter(right_raw)
        right_fit = fit_and_resample(right_clean, n_samples=n_samples)
        weld_points.extend(right_fit.tolist())
        print(f"Right: {len(right_raw)} raw -> {len(right_clean)} MAD -> "
              f"{len(right_fit)} arc samples  "
              f"x=[{right_fit[0,0]:.1f}, {right_fit[-1,0]:.1f}]")

    cloud = pv.PolyData(np.array(all_points))
    welds = pv.PolyData(np.array(weld_points)) if weld_points else pv.PolyData()

    plotter = pv.Plotter()
    plotter.add_mesh(cloud, color='white', point_size=2,
                     render_points_as_spheres=True)
    if welds.n_points > 0:
        plotter.add_mesh(welds, color='red', point_size=10,
                         render_points_as_spheres=True)
    plotter.set_background('black')
    plotter.show()


if __name__ == "__main__":
    main()