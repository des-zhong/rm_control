"""One independent process per dog. Reads /motions, never publishes ROS commands."""
import argparse
import json
import math
import os
from pathlib import Path
import queue
import shlex
import subprocess
import threading
import time

from .core import Limits, Tracker, finite, select_team
from .transport import ssh_environment, ssh_options


def arguments(default_ip, default_user, default_remote):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ip', default=default_ip)
    p.add_argument('--user', default=default_user)
    p.add_argument('--remote-agent', default=default_remote)
    p.add_argument('--interface', default='eth10')
    p.add_argument('--side', choices=['lower', 'upper'], default='lower')
    p.add_argument('--initial-x', type=float, default=0.0)
    p.add_argument('--initial-y', type=float)
    p.add_argument('--initial-yaw-deg', type=float)
    p.add_argument('--xmin', type=float, default=-2.3)
    p.add_argument('--xmax', type=float, default=2.3)
    p.add_argument('--topic', default='/motions')
    p.add_argument('--log-dir', default='run_data/as2_follow')
    p.add_argument('--enable-motion', action='store_true', help='Default is read-only/dry run')
    p.add_argument('--confirm-field-x', action='store_true', help='Measured long axis is mocap X')
    p.add_argument('--accept-unstamped-mocap', action='store_true',
                   help='Acknowledge existing /motions cannot prove source freshness/validity')
    args = p.parse_args()
    if args.initial_y is None:
        args.initial_y = -2.5 if args.side == 'lower' else 2.5
    if args.initial_yaw_deg is None:
        args.initial_yaw_deg = 90.0 if args.side == 'lower' else -90.0
    values = [args.initial_x, args.initial_y, args.initial_yaw_deg, args.xmin, args.xmax]
    if not finite(values) or not args.xmin < args.xmax or not args.xmin <= args.initial_x <= args.xmax:
        p.error('Invalid calibration or X bounds')
    if (args.side == 'lower' and args.initial_y >= 0) or (args.side == 'upper' and args.initial_y <= 0):
        p.error('Lower Y must be negative; upper Y must be positive')
    if args.ip.startswith('-') or args.user.startswith('-'):
        p.error('Invalid SSH destination')
    if args.enable_motion and not (args.confirm_field_x and args.accept_unstamped_mocap):
        p.error('Motion needs --confirm-field-x and --accept-unstamped-mocap; see README')
    return args


def read_states(pipe, latest):
    for line in iter(pipe.readline, b''):
        try:
            message = json.loads(line)
            if message.get('kind') != 'state':
                continue
            stamp = time.monotonic()
            try:
                latest.get_nowait()
            except queue.Empty:
                pass
            latest.put_nowait((stamp, message))
        except (ValueError, AttributeError, queue.Full):
            continue


def main(default_ip='172.20.10.13', default_user='unitree',
         default_remote='/home/unitree/as2_follow/build/as2_agent'):
    args = arguments(default_ip, default_user, default_remote)
    # Lazy ROS imports keep geometry/tests usable without ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from ros2_interfaces.msg import Motions

    directory = Path(args.log_dir).expanduser() / (time.strftime('%Y%m%d_%H%M%S') + '_' + str(time.time_ns()))
    directory.mkdir(parents=True)
    (directory / 'metadata.json').write_text(json.dumps(vars(args), indent=2))
    remote = [args.remote_agent, args.interface]
    if args.enable_motion:
        remote.append('--enable')
    with ssh_environment() as env, \
            (directory / 'ssh.stderr.log').open('wb') as stderr, \
            (directory / 'telemetry.jsonl').open('w', buffering=1) as log:
        process = subprocess.Popen(['ssh', *ssh_options(), args.user+'@'+args.ip,
                                    shlex.join(remote)], stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=stderr,
                                   env=env, start_new_session=True, bufsize=0)
        os.set_blocking(process.stdin.fileno(), False)
        latest = queue.Queue(maxsize=1)
        threading.Thread(target=read_states, args=(process.stdout, latest), daemon=True).start()
        rclpy.init(args=[])

        class Follow(Node):
            def __init__(self):
                super().__init__('as2_follow_' + args.side)
                self.tracker = Tracker((args.initial_x, args.initial_y, math.radians(args.initial_yaw_deg)),
                                       Limits(xmin=args.xmin, xmax=args.xmax))
                self.team = None
                self.poses = None
                self.received = -math.inf
                self.changed = -math.inf
                self.fingerprint = None
                self.state = None
                self.state_time = -math.inf
                self.last_token = -1
                self.fault = None
                self.previous_time = time.monotonic()
                self.last_status = None
                self.subscription = self.create_subscription(Motions, args.topic, self.motion, qos_profile_sensor_data)
                self.timer = self.create_timer(0.05, self.tick)
                self.get_logger().info('AS2 %s; %s; log: %s' % (
                    args.side, 'MOTION ENABLED' if args.enable_motion else 'READ ONLY', directory))

            def motion(self, msg):
                now = time.monotonic()
                poses = [(p.x, p.y, p.theta) for p in msg.poses]
                if len(poses) < 7 or not all(finite(p) for p in poses[:7]):
                    self.poses = None
                    return
                fingerprint = tuple(poses[:7])
                if fingerprint != self.fingerprint:
                    self.changed = now
                    self.fingerprint = fingerprint
                if self.team is None:
                    try:
                        self.team = select_team(poses, args.side)
                    except ValueError as exc:
                        if self.last_status != str(exc):
                            self.get_logger().warning(str(exc))
                            self.last_status = str(exc)
                        return
                    self.get_logger().info('Locked team: ' + ', '.join('RM%d' % i for i in self.team))
                self.poses, self.received = poses, now

            def tick(self):
                now = time.monotonic()
                dt, self.previous_time = now-self.previous_time, now
                try:
                    self.state_time, self.state = latest.get_nowait()
                except queue.Empty:
                    pass
                command = self.tracker.stop() if self.fault else (0., 0., 0.)
                pose, target = None, None
                status = 'waiting for state'
                state = self.state
                if process.poll() is not None:
                    self.get_logger().error('SSH/agent exited; remote watchdog stops motion. See ssh.stderr.log')
                    rclpy.shutdown()
                    return
                if state is not None:
                    try:
                        if state.get('fault'):
                            raise ValueError('Remote agent fault; restart required')
                        if state.get('enabled') != args.enable_motion:
                            raise ValueError('Remote enable state mismatch')
                        odom = state['odom']
                        if len(odom) != 3 or not finite(odom):
                            raise ValueError('Invalid odometry')
                        if not state['valid'] or now-self.state_time > 0.25:
                            status = 'odometry stale'
                        else:
                            pose = self.tracker.world_pose(tuple(odom))
                            if self.fault:
                                status = self.fault
                            elif self.team is None or self.poses is None or now-self.received > 0.25:
                                status = 'mocap missing/stale'
                            elif now-self.changed > 0.6:
                                status = 'mocap unchanged >0.6s (conservative freeze guard)'
                            else:
                                target = sum(self.poses[i][0] for i in self.team)/3
                                command = self.tracker.command(pose, target, self.poses[0][:2], dt)
                                status = 'tracking' if args.enable_motion else 'preview'
                    except (ValueError, KeyError, TypeError) as exc:
                        self.fault = status = str(exc)
                if status not in ('tracking', 'preview'):
                    command = self.tracker.stop()
                if state is not None and state['token'] != self.last_token and now-self.state_time < 0.25:
                    packet = ('%d %.8f %.8f %.8f\n' % (state['token'], *command)).encode()
                    try:
                        n = os.write(process.stdin.fileno(), packet)
                        if n != len(packet):
                            self.fault = 'Partial SSH write; remote watchdog expires command'
                        self.last_token = state['token']
                    except BlockingIOError:
                        self.fault = 'SSH backpressure; restart required'
                    except BrokenPipeError:
                        self.fault = 'SSH closed'
                if status != self.last_status:
                    self.get_logger().info(status)
                    self.last_status = status
                log.write(json.dumps(dict(kind='tracking', unix_ns=time.time_ns(),
                    monotonic=now, side=args.side, team=self.team, state=state, pose=pose,
                    target_x=target, ball=self.poses[0][:2] if self.poses else None,
                    motions=self.poses, command_body=command, status=status), allow_nan=False) + '\n')

        node = Follow()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            # EOF reaches agent; independent remote watchdog also covers lost links.
            process.stdin.close()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
