#!/usr/bin/env python3
"""STRIDE LO launcher: one window with every demo option, Launch / Stop, and the log.

  VS Code status bar "STRIDE: Launch" (or scripts/launcher): starts the shared screen
  (scripts/screen, DISPLAY :99, watch it at http://localhost:6080) and opens this window there,
  so this window, Gazebo and RViz are visible to you (browser) and Claude (scripts/screen shot).

Modes
  Simulation   ros2 launch lo_sim sim.launch.py (synthetic Mid-360 + IMU, optional Gazebo world)
  Replay bag   ros2 bag play --clock BAG + stride_lo.launch.py + lo_graph_demo (+ RViz/lo_viz);
               the bag's own STRIDE/graph/viz outputs are not replayed, so the live ones are
               what you see.

Stop runs scripts/stop_all.sh (also the VS Code "STRIDE: Stop" button): SIGINT to ros2 launch /
bag, then the whole process group, SIGKILL only if it hangs: nothing is left running. The running
group id is kept in data/screen/launch.pgid.
"""
import os
import queue
import re
import shlex
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import filedialog, ttk

SCENARIOS = ['crowd', 'room', 'warehouse', 'corridor', 'open_field', 'stairs', 'turn180']
ROBOTS = ['stride_rig', 'anymal_c', 'default']
GT_TOPICS = ['/ground_truth/odom', '/sim/raw/odom', '(none)']
IMU_TOPICS = ['/livox/imu', '/imu/data']
WS = os.environ.get('STRIDE_WS') or os.path.abspath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..'))
BAG_DIRS = [d for d in ('/data/bags', os.path.join(WS, 'data'), WS) if os.path.isdir(d)]
SKIP_REPLAY = r'^/(stride_lo|lo_graph_demo|lo_viz)/.*|^/clock$'
PGID_FILE = os.path.join(WS, 'data', 'screen', 'launch.pgid')
RUN_FG = os.path.join(WS, 'scripts', 'run_fg')   # SIGINT back to default (see scripts/run_fg)


def bag_topics(path):
    try:
        with open(os.path.join(path, 'metadata.yaml')) as f:
            return set(re.findall(r'name:\s*(/\S+)', f.read()))
    except OSError:
        return set()


class Launcher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('STRIDE LO launcher')
        self.geometry('900x720')
        self.minsize(760, 560)
        self.proc = None
        self.lines = queue.Queue()
        self.v = {
            'mode': tk.StringVar(value='sim'),
            'scenario': tk.StringVar(value='crowd'), 'seed': tk.StringVar(value='1'),
            'rate': tk.StringVar(value='1.0'),
            'gazebo': tk.BooleanVar(value=True), 'gui': tk.BooleanVar(value=True),
            'rviz': tk.BooleanVar(value=True), 'lo': tk.BooleanVar(value=True),
            'record': tk.BooleanVar(value=False),
            'bag': tk.StringVar(value=''), 'robot': tk.StringVar(value='stride_rig'),
            'gt': tk.StringVar(value='/ground_truth/odom'), 'imu': tk.StringVar(value='/livox/imu'),
            'play_rate': tk.StringVar(value='1.0'), 'rviz_bag': tk.BooleanVar(value=True),
            'graph': tk.BooleanVar(value=True), 'base_tf': tk.BooleanVar(value=True),
        }
        self._build()
        self.v['gazebo'].trace_add('write', lambda *_: self._sync())
        self.v['mode'].trace_add('write', lambda *_: self._sync())
        self._sync()
        self.protocol('WM_DELETE_WINDOW', self.on_close)
        self.after(100, self._pump)

    # ---- layout ----------------------------------------------------------------------------
    def _build(self):
        pad = {'padx': 6, 'pady': 3}
        top = ttk.Frame(self)
        top.pack(fill='x', **pad)
        ttk.Label(top, text='Mode:').pack(side='left')
        ttk.Radiobutton(top, text='Simulation', value='sim', variable=self.v['mode']).pack(side='left')
        ttk.Radiobutton(top, text='Replay a bag', value='bag', variable=self.v['mode']).pack(side='left')

        self.sim = ttk.LabelFrame(self, text='Simulation (synthetic Livox Mid-360 + IMU, ground truth)')
        self.sim.pack(fill='x', **pad)
        r = 0
        ttk.Label(self.sim, text='Scenario').grid(row=r, column=0, sticky='w', **pad)
        ttk.Combobox(self.sim, textvariable=self.v['scenario'], values=SCENARIOS, width=14,
                     state='readonly').grid(row=r, column=1, sticky='w', **pad)
        ttk.Label(self.sim, text='Seed').grid(row=r, column=2, sticky='e', **pad)
        ttk.Spinbox(self.sim, textvariable=self.v['seed'], from_=1, to=999, width=6).grid(
            row=r, column=3, sticky='w', **pad)
        ttk.Label(self.sim, text='Rate').grid(row=r, column=4, sticky='e', **pad)
        self.rate_box = ttk.Spinbox(self.sim, textvariable=self.v['rate'], from_=0.1, to=4.0,
                                    increment=0.1, width=6)
        self.rate_box.grid(row=r, column=5, sticky='w', **pad)
        r += 1
        ttk.Checkbutton(self.sim, text='Gazebo world (3D view)',
                        variable=self.v['gazebo']).grid(row=r, column=0, columnspan=2, sticky='w', **pad)
        self.gui_box = ttk.Checkbutton(self.sim, text='Gazebo window',
                                       variable=self.v['gui'])
        self.gui_box.grid(row=r, column=2, columnspan=2, sticky='w', **pad)
        ttk.Checkbutton(self.sim, text='RViz', variable=self.v['rviz']).grid(
            row=r, column=4, sticky='w', **pad)
        r += 1
        ttk.Checkbutton(self.sim, text='STRIDE + GTSAM graph demo', variable=self.v['lo']).grid(
            row=r, column=0, columnspan=2, sticky='w', **pad)
        ttk.Checkbutton(self.sim, text='Record a bag (into data/)', variable=self.v['record']).grid(
            row=r, column=2, columnspan=2, sticky='w', **pad)

        self.bag = ttk.LabelFrame(self, text='Replay a recorded bag (STRIDE runs live on it)')
        self.bag.pack(fill='x', **pad)
        ttk.Label(self.bag, text='Bag').grid(row=0, column=0, sticky='w', **pad)
        ttk.Entry(self.bag, textvariable=self.v['bag'], width=60).grid(
            row=0, column=1, columnspan=4, sticky='we', **pad)
        ttk.Button(self.bag, text='Browse…', command=self.pick_bag).grid(row=0, column=5, **pad)
        ttk.Label(self.bag, text='Robot').grid(row=1, column=0, sticky='w', **pad)
        ttk.Combobox(self.bag, textvariable=self.v['robot'], values=ROBOTS, width=12,
                     state='readonly').grid(row=1, column=1, sticky='w', **pad)
        ttk.Label(self.bag, text='Ground truth').grid(row=1, column=2, sticky='e', **pad)
        ttk.Combobox(self.bag, textvariable=self.v['gt'], values=GT_TOPICS, width=18).grid(
            row=1, column=3, sticky='w', **pad)
        ttk.Label(self.bag, text='IMU').grid(row=1, column=4, sticky='e', **pad)
        ttk.Combobox(self.bag, textvariable=self.v['imu'], values=IMU_TOPICS, width=12).grid(
            row=1, column=5, sticky='w', **pad)
        ttk.Label(self.bag, text='Play rate').grid(row=2, column=0, sticky='w', **pad)
        ttk.Spinbox(self.bag, textvariable=self.v['play_rate'], from_=0.1, to=4.0, increment=0.1,
                    width=6).grid(row=2, column=1, sticky='w', **pad)
        ttk.Checkbutton(self.bag, text='RViz', variable=self.v['rviz_bag']).grid(
            row=2, column=2, sticky='w', **pad)
        ttk.Checkbutton(self.bag, text='GTSAM graph demo', variable=self.v['graph']).grid(
            row=2, column=3, sticky='w', **pad)
        ttk.Checkbutton(self.bag, text='Viz publishes base TF', variable=self.v['base_tf']).grid(
            row=2, column=4, columnspan=2, sticky='w', **pad)

        bar = ttk.Frame(self)
        bar.pack(fill='x', **pad)
        self.go = ttk.Button(bar, text='▶  Launch', command=self.launch)
        self.go.pack(side='left')
        self.stop_btn = ttk.Button(bar, text='■  Stop', command=self.stop, state='disabled')
        self.stop_btn.pack(side='left', padx=6)
        self.status = ttk.Label(bar, text='stopped')
        self.status.pack(side='left', padx=12)
        self.cmd = ttk.Label(self, text='', foreground='#666', wraplength=860, justify='left')
        self.cmd.pack(fill='x', **pad)

        legend = ('RViz: green = ground truth, orange = STRIDE odometry, blue = GTSAM graph, '
                  'colour cloud = STRIDE local map (by height), white = current scan, '
                  'text above the robot = STRIDE state, factor health, error vs ground truth.')
        ttk.Label(self, text=legend, wraplength=860, justify='left').pack(fill='x', **pad)
        logf = ttk.Frame(self)
        logf.pack(fill='both', expand=True, **pad)
        self.log = tk.Text(logf, height=18, wrap='none', font=('monospace', 9))
        sb = ttk.Scrollbar(logf, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        sb.pack(side='right', fill='y')
        self.log.pack(side='left', fill='both', expand=True)

    def _sync(self):
        sim = self.v['mode'].get() == 'sim'
        for frame, on in ((self.sim, sim), (self.bag, not sim)):
            for w in frame.winfo_children():
                try:
                    w.configure(state='normal' if on else 'disabled')
                except tk.TclError:
                    pass
        if sim:
            self.gui_box.configure(state='normal' if self.v['gazebo'].get() else 'disabled')

    # ---- bag defaults ----------------------------------------------------------------------
    def pick_bag(self):
        path = filedialog.askdirectory(initialdir=BAG_DIRS[0] if BAG_DIRS else WS,
                                       title='Bag directory (with metadata.yaml)')
        if not path:
            return
        self.v['bag'].set(path)
        topics = bag_topics(path)
        if not topics:
            self.append(f'[launcher] no metadata.yaml in {path}\n')
            return
        if '/sim/raw/odom' in topics:
            self.v['gt'].set('/sim/raw/odom')
            self.v['robot'].set('anymal_c')
        elif '/ground_truth/odom' in topics:
            self.v['gt'].set('/ground_truth/odom')
            self.v['robot'].set('stride_rig')
        else:
            self.v['gt'].set('(none)')
        self.v['imu'].set('/livox/imu' if '/livox/imu' in topics else
                          ('/imu/data' if '/imu/data' in topics else self.v['imu'].get()))
        self.v['base_tf'].set('/tf' not in topics)
        missing = [t for t in ('/livox/lidar', self.v['imu'].get()) if t not in topics]
        self.append(f'[launcher] {os.path.basename(path)}: {len(topics)} topics; robot '
                    f'{self.v["robot"].get()}, GT {self.v["gt"].get()}, IMU {self.v["imu"].get()}'
                    + (f'; MISSING {missing}' if missing else '') + '\n')

    # ---- commands --------------------------------------------------------------------------
    def command(self):
        b = lambda k: 'true' if self.v[k].get() else 'false'   # noqa: E731
        if self.v['mode'].get() == 'sim':
            args = [RUN_FG, 'ros2', 'launch', 'lo_sim', 'sim.launch.py',
                    f'scenario:={self.v["scenario"].get()}', f'seed:={int(self.v["seed"].get())}',
                    f'gazebo:={b("gazebo")}', f'gui:={b("gui") if self.v["gazebo"].get() else "false"}',
                    f'rviz:={b("rviz")}', f'lo:={b("lo")}', f'record:={b("record")}',
                    f'rate:={float(self.v["rate"].get())}']
            return shlex.join(args)
        bag = self.v['bag'].get()
        if not os.path.isdir(bag):
            raise ValueError('choose a bag directory first')
        gt = self.v['gt'].get()
        gt = '' if gt == '(none)' else gt
        node = shlex.join([RUN_FG, 'ros2', 'launch', 'lo_ros', 'stride_lo.launch.py',
                           f'robot:={self.v["robot"].get()}', 'use_sim_time:=true',
                           f'requests:={b("graph")}', f'rviz:={b("rviz_bag")}',
                           f'gt_topic:={gt}', f'viz_base_tf:={b("base_tf")}', 'respawn:=false'])
        parts = [node + ' &']
        if self.v['graph'].get():
            parts.append(shlex.join([RUN_FG, 'ros2', 'run', 'lo_gtsam_adapter', 'lo_graph_demo',
                                     '--ros-args', '-p', 'use_sim_time:=true',
                                     '-p', f'imu_topic:={self.v["imu"].get()}']) + ' &')
        parts.append('sleep 4')
        parts.append(shlex.join([RUN_FG, 'ros2', 'bag', 'play', bag, '--clock', '-r',
                                 str(float(self.v['play_rate'].get())), '-x', SKIP_REPLAY]))
        parts.append('echo "[launcher] bag finished; STRIDE and RViz stay up until Stop"; wait')
        return '\n'.join(parts)

    def launch(self):
        if self.proc:
            return
        try:
            script = self.command()
        except ValueError as e:
            self.append(f'[launcher] {e}\n')
            return
        cwd = os.path.join(WS, 'data')
        os.makedirs(cwd, exist_ok=True)
        self.log.delete('1.0', 'end')
        self.cmd.configure(text=script.replace('\n', '   ;   '))
        self.append(f'[launcher] {script}\n')
        self.proc = subprocess.Popen(['bash', '-c', script], cwd=cwd, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, bufsize=1,
                                     start_new_session=True)
        self._write_pgid(self.proc.pid)
        threading.Thread(target=self._reader, args=(self.proc,), daemon=True).start()
        self.go.configure(state='disabled')
        self.stop_btn.configure(state='normal')
        self.status.configure(text='running')

    def _reader(self, proc):
        for line in proc.stdout:
            self.lines.put(line)
        code = proc.wait()
        self.lines.put(('exit', code))

    def stop(self):
        """scripts/stop_all.sh: SIGINT to ros2 launch / bag first (so launch stops its nodes in
        order and the recorder finalises its bag), then the group, SIGKILL only if it hangs."""
        if not self.proc:
            return
        self.status.configure(text='stopping…')
        stopper = subprocess.Popen([os.path.join(WS, 'scripts', 'stop_all.sh')],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        def relay():
            for line in stopper.stdout:
                self.lines.put('[stop] ' + line)
        threading.Thread(target=relay, daemon=True).start()

    @staticmethod
    def _group_alive(pgid):
        try:
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False

    @staticmethod
    def _write_pgid(pgid):
        try:
            if pgid is None:
                os.remove(PGID_FILE)
            else:
                os.makedirs(os.path.dirname(PGID_FILE), exist_ok=True)
                with open(PGID_FILE, 'w') as f:
                    f.write(f'{pgid}\n')
        except OSError:
            pass

    # ---- log -------------------------------------------------------------------------------
    def append(self, text):
        self.log.insert('end', text)
        if int(self.log.index('end-1c').split('.')[0]) > 5000:
            self.log.delete('1.0', '1000.0')
        self.log.see('end')

    def _pump(self):
        try:
            while True:
                item = self.lines.get_nowait()
                if isinstance(item, tuple):
                    pgid = self.proc.pid if self.proc else None
                    if pgid and self._group_alive(pgid):
                        os.killpg(pgid, signal.SIGKILL)     # stragglers of a finished launch
                    self.append(f'[launcher] exited with code {item[1]}\n')
                    self.proc = None
                    self._write_pgid(None)
                    self.go.configure(state='normal')
                    self.stop_btn.configure(state='disabled')
                    self.status.configure(text=f'stopped (exit {item[1]})')
                else:
                    self.append(item)
        except queue.Empty:
            pass
        self.after(100, self._pump)

    def on_close(self):
        if self.proc:
            self.stop()
            self.after(500, self._close_when_done)
        else:
            self.destroy()

    def _close_when_done(self):
        if self.proc and self._group_alive(self.proc.pid):
            self.after(500, self._close_when_done)
        else:
            self.destroy()


if __name__ == '__main__':
    Launcher().mainloop()
