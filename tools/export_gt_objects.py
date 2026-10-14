#!/usr/bin/env python3
"""Export ground-truth object poses from the running Gazebo world (evaluation only, plan §31-32).

    python3 tools/export_gt_objects.py --world indoor_v1 --out simulation/worlds/indoor_v1_gt.json

Uses `gz model -m <name> -p` for every model listed in simulation/worlds/<world>_objects.json and writes
{id, class, model, position[x,y,z], yaw}. Needs GZ_IP=127.0.0.1 (same as PX4's Gazebo).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def model_pose(name: str):
    out = subprocess.run(['gz', 'model', '-m', name, '-p'], capture_output=True, text=True, timeout=20).stdout
    m = re.search(r'\[(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\]\s*\n\s*-\s*\[(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\]', out)
    if not m:
        m2 = re.findall(r'-?\d+\.\d+', out)
        if len(m2) >= 6:
            vals = [float(v) for v in m2[:6]]
            return vals[:3], vals[5]
        return None, None
    vals = [float(v) for v in m.groups()]
    return vals[:3], vals[5]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='indoor_v1')
    ap.add_argument('--out', default=None)
    a = ap.parse_args(argv)
    os.environ.setdefault('GZ_IP', '127.0.0.1')
    src = os.path.join(ROOT, 'simulation', 'worlds', f'{a.world}_objects.json')
    spec = json.load(open(src))
    gt = {'world': a.world, 'frame': 'map (Gazebo world frame)', 'objects': []}
    for o in spec['objects']:
        pos, yaw = model_pose(o['id'])
        entry = {'id': o['id'], 'class': o['class'], 'model': o['model'], 'position': pos, 'yaw': yaw}
        gt['objects'].append(entry)
        print(f'{o["id"]:<14} {o["class"]:<13} {pos} yaw={yaw if yaw is None else round(math.degrees(yaw))}')
    out = a.out or os.path.join(ROOT, 'simulation', 'worlds', f'{a.world}_gt.json')
    json.dump(gt, open(out, 'w'), indent=2)
    print('written', out)
    return 0 if all(o['position'] for o in gt['objects']) else 1


if __name__ == '__main__':
    sys.exit(main())
