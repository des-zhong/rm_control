"""Supervise two isolated followers; one child's failure does not stop its peer."""
import argparse
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from .transport import ssh_environment


def supervise(commands, env):
    stopping = threading.Event()
    old = {}
    children = {}
    result = 0
    for sig in (signal.SIGINT, signal.SIGTERM):
        old[sig] = signal.signal(sig, lambda *_: stopping.set())
    try:
        for side, command in commands.items():
            if stopping.is_set():
                break
            try:
                children[side] = subprocess.Popen(command, env=env, start_new_session=True)
                print('Started %s follower (pid %d)' % (side, children[side].pid), flush=True)
            except OSError as exc:
                print('Could not start %s: %s; other follower is independent' % (side, exc), flush=True)
                result = 1
        live = dict(children)
        while live and not stopping.wait(.1):
            for side, process in list(live.items()):
                code = process.poll()
                if code is not None:
                    print('%s follower exited (%d); other follower continues' % (side, code), flush=True)
                    del live[side]
                    result = 1
    finally:
        # The children own their SSH connections and close stdin on KeyboardInterrupt.
        for side, process in children.items():
            if process.poll() is None:
                print('Stopping %s follower...' % side, flush=True)
                process.send_signal(signal.SIGINT)
        deadline = time.monotonic()+8
        for process in children.values():
            try:
                process.wait(timeout=max(.01, deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                process.terminate()
        for process in children.values():
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                result = 1
        for sig, handler in old.items():
            signal.signal(sig, handler)
    return result


def main(lower_ip='172.20.10.13', upper_ip='172.20.10.14', user='unitree'):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--lower-ip', default=lower_ip)
    p.add_argument('--upper-ip', default=upper_ip)
    p.add_argument('--user', default=user)
    p.add_argument('--interface', default='eth10')
    p.add_argument('--topic', default='/motions')
    p.add_argument('--log-dir', default='run_data/as2_follow')
    p.add_argument('--enable-motion', action='store_true')
    p.add_argument('--confirm-field-x', action='store_true')
    p.add_argument('--accept-unstamped-mocap', action='store_true')
    args = p.parse_args()
    if args.lower_ip == args.upper_ip:
        p.error('Two followers must connect to different robot IPs')
    if any(v.startswith('-') for v in (args.lower_ip, args.upper_ip, args.user)):
        p.error('Invalid SSH destination')
    if args.enable_motion and not (args.confirm_field_x and args.accept_unstamped_mocap):
        p.error('Motion needs --confirm-field-x and --accept-unstamped-mocap; see README')
    script = Path(__file__).resolve().parents[1]/'run_follow.py'
    commands = {}
    for side, ip in (('lower', args.lower_ip), ('upper', args.upper_ip)):
        command = [sys.executable, '-u', str(script), '--ip', ip, '--side', side,
                   '--user', args.user, '--interface', args.interface, '--topic', args.topic,
                   '--log-dir', str(Path(args.log_dir)/side)]
        for option in ('enable_motion', 'confirm_field_x', 'accept_unstamped_mocap'):
            if getattr(args, option):
                command.append('--'+option.replace('_', '-'))
        commands[side] = command
    # Askpass helper stays alive until both children and their SSH connections finish.
    with ssh_environment() as env:
        return supervise(commands, env)
