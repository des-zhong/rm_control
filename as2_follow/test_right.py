#!/usr/bin/env python3
# Connection arguments are intentionally at the front of the executable file.
DEFAULT_IP = '172.20.10.13'
DEFAULT_USER = 'unitree'
DEFAULT_AGENT = '/home/unitree/as2_follow/build/as2_agent'

import argparse
import json
import os
from pathlib import Path
import queue
import shlex
import signal
import subprocess
import threading
import time

from as2_follow.transport import ssh_environment, ssh_options


def run():
    parser = argparse.ArgumentParser(description='No-mocap body-right sidestep and clockwise yaw test.')
    parser.add_argument('--ip', default=DEFAULT_IP)
    parser.add_argument('--user', default=DEFAULT_USER)
    parser.add_argument('--remote-agent', default=DEFAULT_AGENT)
    parser.add_argument('--interface', default='eth10')
    parser.add_argument('--seconds', type=float, default=5., help='Motion duration; max 10 seconds')
    parser.add_argument('--link-timeout', type=float, default=1.0,
                        help='Local telemetry wait limit, 0.5..2 s; remote 0.25 s watchdog is unchanged')
    parser.add_argument('--execute', action='store_true', help='Actually enable robot motion')
    parser.add_argument('--log-dir', default='run_data/as2_right_test')
    args = parser.parse_args()
    if not 0 < args.seconds <= 10:
        parser.error('--seconds must be in (0, 10]')
    if not .5 <= args.link_timeout <= 2.:
        parser.error('--link-timeout must be in [0.5, 2]')
    if args.ip.startswith('-') or args.user.startswith('-'):
        parser.error('Invalid SSH destination')
    folder = Path(args.log_dir).expanduser() / str(time.time_ns())
    folder.mkdir(parents=True)
    (folder/'metadata.json').write_text(json.dumps(vars(args), indent=2))
    stop = threading.Event()
    previous = {}
    try:
        with ssh_environment() as env, (folder/'ssh.stderr.log').open('wb') as err, \
                (folder/'telemetry.jsonl').open('w', buffering=1) as log:
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous[sig] = signal.signal(sig, lambda *_: stop.set())
            command = [args.remote_agent, args.interface]
            if args.execute:
                command.append('--enable')
            process = subprocess.Popen(['ssh', *ssh_options(), args.user+'@'+args.ip, shlex.join(command)],
                                       stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err,
                                       start_new_session=True, env=env, bufsize=0)
            os.set_blocking(process.stdin.fileno(), False)
            states = queue.Queue(maxsize=1)
            # Dedicated reader keeps stop acknowledgement separate from state packets.
            acknowledged = threading.Event()
            stop_result = []
            def reader():
                for line in iter(process.stdout.readline, b''):
                    try:
                        msg = json.loads(line)
                        if msg.get('kind') == 'stopped':
                            stop_result.append(msg)
                            acknowledged.set()
                        elif msg.get('kind') == 'state':
                            try:
                                states.get_nowait()
                            except queue.Empty:
                                pass
                            states.put_nowait((time.monotonic(), msg))
                    except (ValueError, AttributeError, queue.Full):
                        continue
            threading.Thread(target=reader, daemon=True).start()
            print(('REAL MOTION' if args.execute else 'READ ONLY') +
                  ': body-right 0.03 m/s + clockwise 0.06 rad/s, 2 s ramp; Ctrl+C stops.', flush=True)
            print('Log: '+str(folder), flush=True)
            print('Waiting for valid state (max 8 seconds)...', flush=True)
            deadline = time.monotonic()+8
            start = None
            last_state = time.monotonic()
            token = -1
            result = 0
            delayed = False
            confirmed = False
            try:
                while not stop.is_set():
                    now = time.monotonic()
                    if process.poll() is not None:
                        raise RuntimeError('SSH/agent exited; see ssh.stderr.log')
                    if start is None and now > deadline:
                        raise RuntimeError('No valid state within 8 seconds')
                    if start is not None and now-start >= args.seconds:
                        print('Duration reached; stopping.', flush=True)
                        break
                    if start is not None and now-last_state > args.link_timeout:
                        raise RuntimeError('State link stale >%.2f s; stopping test' % args.link_timeout)
                    if start is not None and now-last_state > .25 and not delayed:
                        print('State link delayed; no fresh commands sent, remote watchdog remains active.', flush=True)
                        delayed = True
                    try:
                        received, state = states.get(timeout=.025)
                    except queue.Empty:
                        continue
                    if state.get('fault') or state.get('rc') != 0:
                        raise RuntimeError('Remote SDK fault: '+str(state))
                    if state.get('enabled') != args.execute:
                        raise RuntimeError('Remote mode mismatch')
                    if not state.get('valid'):
                        if start is not None:
                            raise RuntimeError('Odometry invalid; stopping test')
                        continue
                    if delayed:
                        print('State link recovered.', flush=True)
                        delayed = False
                    last_state = received
                    if not confirmed and any(abs(v)>1e-5 for v in state.get('applied_body', [])):
                        print('Remote SDK called with nonzero velocity (physical movement needs observation).', flush=True)
                        confirmed = True
                    if now-received > .2 or state['token'] <= token:
                        continue
                    if start is None:
                        start = now
                        print('Test started. Press Ctrl+C at any time.', flush=True)
                    ramp = min(1., (now-start)/2.)
                    velocity = (0., -.03*ramp, -.06*ramp)
                    packet = ('%d %.8f %.8f %.8f\n' % (state['token'], *velocity)).encode()
                    if os.write(process.stdin.fileno(), packet) != len(packet):
                        raise RuntimeError('Partial command write')
                    token = state['token']
                    log.write(json.dumps(dict(unix_ns=time.time_ns(), state=state, command_body=velocity))+'\n')
            except (RuntimeError, OSError) as exc:
                print(str(exc), flush=True)
                result = 1
            finally:
                # STOP bypasses token checks; EOF is a second stop path if this write fails.
                print('Sending STOP...', flush=True)
                try:
                    os.write(process.stdin.fileno(), b'STOP\n')
                except OSError:
                    pass
                process.stdin.close()
                acknowledged.wait(timeout=1.5)
                if stop_result:
                    log.write(json.dumps(stop_result[-1])+'\n')
                if args.execute and acknowledged.is_set() and stop_result[-1].get('rc') == 0:
                    print('StopMove returned success. Confirm the dog has physically stopped.', flush=True)
                elif not args.execute and acknowledged.is_set():
                    print('Read-only agent exited.', flush=True)
                else:
                    print('No successful stop acknowledgement; use remote control emergency stop.', flush=True)
                    result = 1
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
            return result
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(run())
