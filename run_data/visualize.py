#!/usr/bin/env python3
"""Plot a recorder session without ROS: python3 run_data/visualize.py TIMESTAMP."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:
    raise SystemExit('Missing matplotlib. Install with: python3 -m pip install matplotlib')


CLOCK = 'receive_monotonic_ns'


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else float('nan')
    except (ValueError, TypeError):
        return float('nan')


def read_csv(path):
    if not path.exists():
        print(f'Skip missing file: {path.name}')
        return []
    with path.open(encoding='utf-8', newline='') as handle:
        return list(csv.DictReader(handle))


def read_messages(path):
    if not path.exists():
        print('Skip missing file: messages.jsonl')
        return
    with path.open(encoding='utf-8') as handle:
        for line_no, line in enumerate(handle, 1):
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                print(f'Skip invalid JSON at line {line_no}', file=sys.stderr)


def save(fig, folder, name):
    fig.tight_layout()
    path = folder / name
    fig.savefig(path, dpi=160, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_series(series, folder, filename, title, labels, start, step=False):
    if not series:
        return
    fig, axes = plt.subplots(len(labels), 1, figsize=(12, 3 * len(labels)),
                             sharex=True, squeeze=False)
    for name in sorted(series):
        samples = sorted(series[name])
        times = [(s[0] - start) / 1e9 for s in samples]
        for index, ylabel in enumerate(labels):
            ax = axes[index, 0]
            values = [s[index + 1] for s in samples]
            if step:
                ax.step(times, values, where='post', label=name)
            else:
                ax.plot(times, values, label=name, linewidth=1)
            ax.set_ylabel(ylabel)
            ax.grid(alpha=.25)
    axes[0, 0].set_title(title)
    axes[0, 0].legend(loc='best', fontsize='small', ncol=3)
    axes[-1, 0].set_xlabel('Elapsed time (s; recorder monotonic clock)')
    save(fig, folder, filename)


def visualize(folder):
    trajectory = read_csv(folder / 'trajectory.csv')
    events = read_csv(folder / 'events.csv')
    groups = {}
    series = {key: {} for key in ('cmd_vel', 'ctrl', 'rvo2', 'targets',
                                 'grip', 'arm', 'state', 'codes', 'ys')}
    stamps = []
    for row in trajectory:
        stamp = int(row[CLOCK])
        stamps.append(stamp)
        groups.setdefault(row['entity'], []).append((
            stamp, *[number(row[k]) for k in
                     ('x', 'y', 'theta', 'vx_raw', 'vy_raw', 'omega_raw')]))
    for row in events:
        stamps.append(int(row[CLOCK]))
    for record in read_messages(folder / 'messages.jsonl'):
        topic = record.get('topic', '')
        stamp = int(record[CLOCK])
        msg = record.get('message', {})
        stamps.append(stamp)
        def add(kind, name, values):
            series[kind].setdefault(name, []).append((stamp, *map(number, values)))
        if topic.endswith('/cmd_vel'):
            add('cmd_vel', topic.split('/')[1],
                [msg['linear']['x'], msg['linear']['y'], msg['angular']['z']])
        elif topic in ('/ctrl', '/rvo2', '/to_rvo'):
            kind = {'/ctrl': 'ctrl', '/rvo2': 'rvo2', '/to_rvo': 'targets'}[topic]
            for i, pose in enumerate(msg.get('pose', [])):
                if i:
                    add(kind, f'RM{i}', [pose['x'], pose['y'], pose['theta']])
            if topic == '/ctrl':
                for i, code in enumerate(msg.get('code', [])):
                    if i:
                        add('codes', f'RM{i}', [code])
        elif '/grip_vision/' in topic and 'data' in msg:
            add('grip', topic.lstrip('/').replace('/grip_vision/', '/'), [msg['data']])
        elif topic.endswith('/arm_position'):
            add('arm', topic.split('/')[1], [msg['point'][k] for k in ('x', 'y', 'z')])
        elif topic.endswith('/state_id'):
            add('state', topic.split('/')[1], [msg['state']])
        elif topic == '/yscomm':
            for i, value in enumerate(msg.get('comm', []), 1):
                add('ys', f'YS{i}', [value])
    if not stamps:
        raise ValueError('No recorded samples found in this session.')
    start = min(stamps)
    if groups:
        fig, ax = plt.subplots(figsize=(10, 8))
        for entity in sorted(groups):
            samples = sorted(groups[entity])
            x = [r[1] for r in samples]
            y = [r[2] for r in samples]
            line, = ax.plot(x, y, label=entity, linewidth=2 if entity == 'ball' else 1,
                            alpha=.85)
            ax.scatter(x[0], y[0], marker='o', color=line.get_color(), s=30)
            ax.scatter(x[-1], y[-1], marker='x', color=line.get_color(), s=40)
        ax.set(xlabel='x (m)', ylabel='y (m)',
               title='Recorded trajectories (circle=start, cross=end)')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=.25)
        ax.legend(ncol=3, fontsize='small')
        save(fig, folder, 'trajectories.png')
        positions = {name: [(r[0], *r[1:4]) for r in rows]
                     for name, rows in groups.items()}
        plot_series(positions, folder, 'positions.png', 'Recorded positions and heading',
                    ['x (m)', 'y (m)', 'theta (rad)'], start)
        if 'ball' in groups:
            ball = sorted(groups['ball'])
            speed = []
            for previous, current in zip(ball, ball[1:]):
                dt = (current[0] - previous[0]) / 1e9
                if dt > 0:
                    speed.append((current[0], math.hypot(current[1]-previous[1],
                                                       current[2]-previous[2])/dt))
            if speed:
                plot_series({'ball': speed}, folder, 'ball_speed_estimated.png',
                            'Ball speed estimated from positions and receive times (unfiltered)',
                            ['Speed (m/s)'], start)
        raw = {name: [(r[0], *r[4:7]) for r in rows] for name, rows in groups.items()}
        plot_series(raw, folder, 'velocities_raw.png',
                    'Raw recorded velocities (existing mocap scaling retained)',
                    ['vx raw', 'vy raw', 'omega raw'], start)
    specifications = [
        ('cmd_vel', 'cmd_vel.png', 'Commands sent to RM chassis', ['vx (m/s)', 'vy (m/s)', 'wz (rad/s)'], False),
        ('ctrl', 'ctrl.png', 'Final strategy control quantities', ['x component', 'y component', 'angular component'], False),
        ('rvo2', 'rvo2.png', 'Avoidance output quantities', ['x component', 'y component', 'angular component'], False),
        ('targets', 'targets.png', 'Strategy target poses', ['x (m)', 'y (m)', 'theta (rad)'], False),
        ('grip', 'grip_evidence.png', 'Visual grip evidence (confidence is not success probability)', ['Reported value'], True),
        ('arm', 'arm_positions.png', 'Reported arm positions', ['x (m)', 'y (m)', 'z (m)'], False),
        ('state', 'execution_states.png', 'Reported RM execution states', ['State code'], True),
        ('codes', 'command_codes.png', 'RM requested action codes', ['Action code'], True),
        ('ys', 'ys_commands.png', 'Yanshee requests (0=standby, 1=prepare, 2=kick)', ['Command'], True),
    ]
    for kind, filename, title, labels, step in specifications:
        plot_series(series[kind], folder, filename, title, labels, start, step)
    relevant = sorted(
        (e for e in events if e['entity'] not in ('topic', 'recorder')),
        key=lambda e: int(e[CLOCK]))
    # Split dense event tables into pages so every event remains readable.
    for offset in range(0, len(relevant), 35):
        page = relevant[offset:offset+35]
        fig, ax = plt.subplots(figsize=(18, max(3, len(page)*.32+1.5)))
        ax.axis('off')
        cells = [[f"{(int(e[CLOCK])-start)/1e9:.3f}", e['entity'], e['topic'],
                  e['event'], e['previous'], e['value']] for e in page]
        table = ax.table(cellText=cells, colLabels=['Time (s)', 'Entity', 'Topic',
                         'Observed event', 'Previous', 'Value'], loc='center',
                         colWidths=[.07,.06,.22,.43,.11,.11])
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        table.scale(1, 1.4)
        ax.set_title('Stage events: observed requests/status transitions, not physical success')
        save(fig, folder, f'events_{offset//35+1:03d}.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', help='Timestamp folder name or full session path')
    args = parser.parse_args()
    folder = Path(args.session).expanduser()
    if not folder.is_dir():
        folder = Path(__file__).resolve().parent / args.session
    if not folder.is_dir():
        parser.error(f'Session directory does not exist: {args.session}')
    try:
        visualize(folder.resolve())
    except (ValueError, KeyError, OSError) as exc:
        parser.exit(1, f'Cannot visualize session: {exc}\n')


if __name__ == '__main__':
    main()
