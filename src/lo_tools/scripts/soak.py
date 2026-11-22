#!/usr/bin/env python3
"""Soak test (docs/DESIGN.md: 8 h, memory flat within +-5 %) on synthetic data.

  ros2 run lo_tools soak.py [--hours 8] [--scenario room] [--seed 1]

Runs `lo_synth_run SCENARIO --soak-hours H`: ONE pipeline is driven through the closed-loop
scenario back to back in continuous message time until H hours of data were processed. RSS is
sampled after every repetition; the run fails when the RSS spread over the second half exceeds
5 % or any factor is a false accept. The verdict line is printed by lo_synth_run.
"""
import argparse
import shutil
import subprocess
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hours', type=float, default=8.0)
    ap.add_argument('--scenario', default='room')
    ap.add_argument('--seed', type=int, default=1)
    a = ap.parse_args()
    exe = shutil.which('lo_synth_run')
    if exe is None:
        sys.exit('lo_synth_run not on PATH (source the workspace install first)')
    cmd = [exe, a.scenario, '--seed', str(a.seed), '--soak-hours', str(a.hours)]
    print(' '.join(cmd), flush=True)
    return subprocess.call(cmd)


if __name__ == '__main__':
    sys.exit(main())
