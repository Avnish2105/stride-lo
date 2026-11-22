#!/usr/bin/env python3
"""Replay and evaluate a set of bags, then write one table (docs/PLAN.md results).

  ros2 run lo_tools bag_suite.py OUT_DIR BAG [BAG ...] [--robot anymal_c] [--kappa 20]
        [--requests-from /estimator/odometry_keyframe] [--set name=value ...] [--keep]

A BAG may be a bag directory or a directory holding attempt_*/bag. For every bag:
lo_replay --out OUT_DIR/<name>, evaluate.py --json, and a row in OUT_DIR/SUITE.md and
OUT_DIR/suite.json. The output bags are deleted unless --keep (the summaries stay).
Runs are sequential so the timing columns are not distorted.
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import sys


def bag_dirs(paths):
    out = []
    for p in paths:
        if os.path.isfile(os.path.join(p, 'metadata.yaml')):
            out.append(p)
        else:
            out.extend(sorted(glob.glob(os.path.join(p, 'attempt_*', 'bag')),
                              key=lambda s: int(s.split('attempt_')[1].split('_')[0])))
    return out


def name_of(bag):
    parts = os.path.normpath(bag).split(os.sep)
    return '_'.join(parts[-3:-1]) if parts[-1] == 'bag' else parts[-1]


def tool(name):
    prefix = subprocess.run(['ros2', 'pkg', 'prefix', 'lo_tools'], capture_output=True, text=True,
                            check=True).stdout.strip()
    return os.path.join(prefix, 'lib', 'lo_tools', name)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out')
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--robot', default='anymal_c')
    ap.add_argument('--kappa', type=float, default=20.0, help='kappa of the config (for the suggestion)')
    ap.add_argument('--requests-from', default='/estimator/odometry_keyframe')
    ap.add_argument('--request-latency', type=float, default=0.005,
                    help='requests arrive at stamp + this (a backend publishing NodeRequest at node '
                         'creation); < 0: at their bag receive time')
    ap.add_argument('--gt', default='/sim/raw/odom', help='ground truth topic (1 kHz in the GPS-denied bags)')
    ap.add_argument('--set', action='append', default=[])
    ap.add_argument('--keep', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    replay, evaluate = tool('lo_replay'), tool('evaluate.py')
    rows = []
    for bag in bag_dirs(a.bags):
        name = name_of(bag)
        out = os.path.join(a.out, name)
        cmd = [replay, '--bag', bag, '--robot', a.robot, '--out', out, '--requests-from', a.requests_from,
               '--request-latency', str(a.request_latency), '--gt', a.gt]
        for kv in a.set:
            cmd += ['--set', kv]
        print(f'== {name}', flush=True)
        rp = subprocess.run(cmd, capture_output=True, text=True)
        print(rp.stdout.strip().splitlines()[-1] if rp.stdout.strip() else rp.stderr[-500:], flush=True)
        if rp.returncode != 0:
            rows.append({'bag': name, 'error': rp.stderr[-300:]})
            continue
        with open(os.path.join(out, 'summary.json')) as f:
            summ = json.load(f)
        js = os.path.join(a.out, f'{name}.eval.json')
        ev = subprocess.run([sys.executable, evaluate, out, '--gt', a.gt, '--kappa', str(a.kappa), '--json', js],
                            capture_output=True, text=True)
        print(ev.stdout.strip(), flush=True)
        met = {}
        if os.path.exists(js):
            with open(js) as f:
                met = json.load(f)
        shutil.copy(os.path.join(out, 'summary.json'), os.path.join(a.out, f'{name}.summary.json'))
        if not a.keep:
            shutil.rmtree(out, ignore_errors=True)
        rows.append({'bag': name, **summ, **met})

    with open(os.path.join(a.out, 'suite.json'), 'w') as f:
        json.dump(rows, f, indent=2)
    ok = [r for r in rows if 'error' not in r and 'rpe_trans_pct' in r]

    def col(r, k, fmt):
        v = r.get(k)
        return fmt.format(v) if isinstance(v, (int, float)) else '-'

    lines = ['| bag | path m | RPE % | median % | max % | deg/m | FAST-LIO % | FAST-LIO deg/m | NEES | '
             'false acc. | avail. | rejected | losts | t_med ms | t_p99 ms |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for r in rows:
        if 'error' in r:
            lines.append(f"| {r['bag']} | replay failed | | | | | | | | | | | | | |")
            continue
        lines.append(' | '.join([
            f"| {r['bag']}", col(r, 'path_length_m', '{:.0f}'), col(r, 'rpe_trans_pct', '{:.3f}'),
            col(r, 'rpe_trans_median_pct', '{:.3f}'), col(r, 'rpe_trans_max_pct', '{:.2f}'),
            col(r, 'rpe_rot_deg_per_m', '{:.4f}'), col(r, 'fastlio_rpe_trans_pct', '{:.3f}'),
            col(r, 'fastlio_rpe_rot_deg_per_m', '{:.4f}'), col(r, 'mean_nees', '{:.2f}'),
            col(r, 'false_accepts', '{:d}'), col(r, 'availability', '{:.3f}'), col(r, 'rejected', '{:d}'),
            col(r, 'losts', '{:d}'), col(r, 'timing_median_ms', '{:.1f}'),
            col(r, 'timing_p99_ms', '{:.1f}')]) + ' |')
    if ok:
        def mean(k):
            v = [r[k] for r in ok if isinstance(r.get(k), (int, float))]
            return sum(v) / len(v) if v else float('nan')
        fa = sum(int(r.get('false_accepts', 0)) for r in ok)
        lines.append(f"| **mean of {len(ok)}** | {mean('path_length_m'):.0f} | **{mean('rpe_trans_pct'):.3f}** | "
                     f"{mean('rpe_trans_median_pct'):.3f} | {max(r['rpe_trans_max_pct'] for r in ok):.2f} | "
                     f"**{mean('rpe_rot_deg_per_m'):.4f}** | {mean('fastlio_rpe_trans_pct'):.3f} | "
                     f"{mean('fastlio_rpe_rot_deg_per_m'):.4f} | **{mean('mean_nees'):.2f}** | **{fa}** | "
                     f"{mean('availability'):.3f} | | | {mean('timing_median_ms'):.1f} | "
                     f"{mean('timing_p99_ms'):.1f} |")
    table = '\n'.join(lines)
    with open(os.path.join(a.out, 'SUITE.md'), 'w') as f:
        f.write(table + '\n')
    print(table)
    return 0 if ok and all(int(r.get('false_accepts', 0)) == 0 for r in ok) else 1


if __name__ == '__main__':
    sys.exit(main())
