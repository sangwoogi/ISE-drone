#!/usr/bin/env python3
"""Plot Phase 1 trajectories recorded by tests/simulation/test_flight_sequence.py.

    python3 tools/plot_flight.py experiments/001_phase1_flight            # all runs -> summary.png
    python3 tools/plot_flight.py experiments/001_phase1_flight --run 3    # one run, 3 panels
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import sys


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    cols = {k: [float(r[k]) for r in rows] for k in ('t', 'x', 'y', 'z', 'sp_x', 'sp_y', 'sp_z')}
    cols['nav_state'] = [r['nav_state'] for r in rows]
    return cols


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('exp_dir')
    ap.add_argument('--run', type=int, help='plot a single run in detail')
    ap.add_argument('--out', help='output PNG (default: <exp_dir>/summary.png or run<N>.png)')
    args = ap.parse_args(argv)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    files = sorted(glob.glob(os.path.join(args.exp_dir, 'trajectories', '*.csv')))
    if not files:
        print('no trajectory CSVs found', file=sys.stderr)
        return 1

    if args.run:
        files = [f for f in files if f.endswith(f'run{args.run:02d}.csv')][-1:]
        d = load(files[0])
        fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        for ax, k in zip(axes, 'xyz'):
            ax.plot(d['t'], d[k], label=f'{k} (estimated)')
            ax.plot(d['t'], d[f'sp_{k}'], '--', label=f'{k} setpoint')
            ax.set_ylabel(f'{k} [m]')
            ax.grid(alpha=0.3)
            ax.legend(loc='upper right')
        axes[-1].set_xlabel('t [s]')
        fig.suptitle(f'Phase 1 run {args.run}: {os.path.basename(files[0])}')
        out = args.out or os.path.join(args.exp_dir, f'run{args.run:02d}.png')
    else:
        fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
        for i, f in enumerate(files):
            d = load(f)
            lbl = os.path.basename(f).split('_')[-1].replace('.csv', '')
            axes[0].plot(d['x'], d['y'], lw=1, label=lbl)
            axes[1].plot(d['t'], d['z'], lw=1, label=lbl)
        d = load(files[-1])
        axes[0].plot(d['sp_x'], d['sp_y'], 'k--', lw=1, label='setpoint (last run)')
        axes[1].plot(d['t'], d['sp_z'], 'k--', lw=1, label='setpoint (last run)')
        axes[0].set_xlabel('x East [m]'); axes[0].set_ylabel('y North [m]'); axes[0].set_aspect('equal')
        axes[0].set_title('Top-down path'); axes[0].grid(alpha=0.3)
        axes[1].set_xlabel('t [s]'); axes[1].set_ylabel('z Up [m]'); axes[1].set_title('Altitude'); axes[1].grid(alpha=0.3)
        axes[1].legend(fontsize=7, ncol=2)
        fig.suptitle(f'Phase 1 flight sequence - {len(files)} runs')
        out = args.out or os.path.join(args.exp_dir, 'summary.png')
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
