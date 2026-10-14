"""slam_eval — ATE / RPE between an estimated and a ground-truth trajectory (CSV: t,x,y,z,...).

    ros2 run drone_slam slam_eval --est est.csv --gt gt.csv [--max-dt 0.05] [--rpe-delta 1.0]

ATE: RMSE of position error after time association (optionally with Umeyama alignment, --align).
RPE: RMSE of relative-motion error over `rpe_delta` seconds.
Baseline implementation with numpy only; Phase 4 adds rosbag extraction and plots.
"""
from __future__ import annotations

import argparse
import csv
import sys

import numpy as np


def load_csv(path):
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r['t']) for r in rows])
    p = np.array([[float(r['x']), float(r['y']), float(r['z'])] for r in rows])
    return t, p


def associate(t_est, t_gt, max_dt):
    idx = np.searchsorted(t_gt, t_est)
    pairs = []
    for i, j in enumerate(idx):
        cands = [k for k in (j - 1, j) if 0 <= k < len(t_gt)]
        if not cands:
            continue
        k = min(cands, key=lambda k: abs(t_gt[k] - t_est[i]))
        if abs(t_gt[k] - t_est[i]) <= max_dt:
            pairs.append((i, k))
    return pairs


def umeyama(src, dst):
    """Rigid (R, t) with unit scale aligning src -> dst."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(3)
    if np.linalg.det(Vt.T @ U.T) < 0:
        D[2, 2] = -1
    R = Vt.T @ D @ U.T
    return R, mu_d - R @ mu_s


def ate(est, gt, align=False):
    if align:
        R, t = umeyama(est, gt)
        est = (R @ est.T).T + t
    err = np.linalg.norm(est - gt, axis=1)
    return float(np.sqrt(np.mean(err ** 2))), float(err.mean()), float(err.max())


def rpe(t, est, gt, delta):
    errs = []
    j = 0
    for i in range(len(t)):
        while j < len(t) and t[j] - t[i] < delta:
            j += 1
        if j >= len(t):
            break
        d_est = np.linalg.norm(est[j] - est[i])
        d_gt = np.linalg.norm(gt[j] - gt[i])
        errs.append(abs(d_est - d_gt))
    if not errs:
        return float('nan'), float('nan')
    errs = np.array(errs)
    return float(np.sqrt(np.mean(errs ** 2))), float(errs.mean())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--est', required=True)
    ap.add_argument('--gt', required=True)
    ap.add_argument('--max-dt', type=float, default=0.05)
    ap.add_argument('--rpe-delta', type=float, default=1.0)
    ap.add_argument('--align', action='store_true', help='Umeyama rigid alignment before ATE')
    a = ap.parse_args(argv)
    t_e, p_e = load_csv(a.est)
    t_g, p_g = load_csv(a.gt)
    pairs = associate(t_e, t_g, a.max_dt)
    if len(pairs) < 2:
        print('not enough associated samples', file=sys.stderr)
        return 1
    ie, ig = zip(*pairs)
    est, gt, t = p_e[list(ie)], p_g[list(ig)], t_e[list(ie)]
    a_rmse, a_mean, a_max = ate(est, gt, a.align)
    r_rmse, r_mean = rpe(t, est, gt, a.rpe_delta)
    print(f'samples={len(pairs)}  ATE rmse={a_rmse:.3f} m mean={a_mean:.3f} max={a_max:.3f}   '
          f'RPE({a.rpe_delta}s) rmse={r_rmse:.3f} m mean={r_mean:.3f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
