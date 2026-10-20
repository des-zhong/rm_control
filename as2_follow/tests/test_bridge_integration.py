"""Real ROS subscription + fake SSH peer: no robot network or SDK commands."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest


@unittest.skipUnless(importlib.util.find_spec('rclpy') and importlib.util.find_spec('ros2_interfaces'),
                     'Source ROS and workspace install/setup.bash first')
class BridgeIntegration(unittest.TestCase):
    def test_peer_exit_isolated(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            fake = root / 'ssh'
            fake.write_text('#!/bin/sh\nexit 3\n')
            fake.chmod(0o700)
            env = dict(os.environ, PATH=tmp+os.pathsep+os.environ['PATH'])
            script = Path(__file__).resolve().parents[1] / 'run_follow.py'
            process = subprocess.run([sys.executable, str(script), '--log-dir', tmp],
                                     env=env, text=True, capture_output=True, timeout=6)
            self.assertEqual(process.returncode, 0, process.stdout+process.stderr)
            self.assertIn('SSH/agent exited', process.stdout+process.stderr)

    def test_preview_tracking_and_stale_stop(self):
        import rclpy
        from geometry_msgs.msg import Pose2D
        from ros2_interfaces.msg import Motions
        rclpy.init()
        node = rclpy.create_node('as2_follow_integration_publisher')
        pub = node.create_publisher(Motions, '/as2_test_motions', 10)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            fake = root / 'ssh'
            fake.write_text('''#!/usr/bin/python3
import json, select, sys, time
n=0
while True:
    ready,_,_=select.select([sys.stdin],[],[],0.05)
    if ready and not sys.stdin.readline(): break
    n+=1
    print(json.dumps(dict(kind='state',token=n,valid=True,odom=[0,0,0],enabled=False,fault=False,rc=0)),flush=True)
''')
            fake.chmod(0o700)
            env = dict(os.environ, PATH=tmp+os.pathsep+os.environ['PATH'])
            script = Path(__file__).resolve().parents[1] / 'run_follow.py'
            with (root/'output.log').open('w') as output:
                process = subprocess.Popen([sys.executable, str(script), '--topic', '/as2_test_motions',
                                            '--log-dir', tmp], env=env, stdout=output, stderr=output)
                try:
                    until = time.monotonic()+4
                    while time.monotonic()<until:
                        ball = .1 + .03*(until-time.monotonic())
                        xs = [-1.5, -1., -.5, .5, 1., 1.5]
                        msg = Motions()
                        msg.poses = [Pose2D(x=ball, y=0., theta=0.)] + [Pose2D(x=x,y=0.,theta=0.) for x in xs]
                        pub.publish(msg)
                        time.sleep(.04)
                    # Keep receiving the same source frame: conservative freeze stop.
                    until = time.monotonic()+.9
                    while time.monotonic()<until:
                        pub.publish(msg)
                        time.sleep(.04)
                    # No new mocap messages: bridge must send zero on receive timeout.
                    time.sleep(.7)
                finally:
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=5)
            files = list(root.glob('*/telemetry.jsonl'))
            self.assertEqual(len(files), 1, (root/'output.log').read_text())
            rows = [json.loads(line) for line in files[0].read_text().splitlines()]
            previews = [r for r in rows if r['status']=='preview']
            self.assertTrue(previews, (root/'output.log').read_text())
            self.assertEqual(previews[-1]['team'], [1,2,3])
            self.assertEqual(previews[-1]['target_x'], -1.)
            self.assertTrue(any(abs(r['command_body'][1])>0 for r in previews))
            frozen = [r for r in rows if 'freeze guard' in r['status']]
            self.assertTrue(frozen)
            self.assertTrue(all(r['command_body']==[0.,0.,0.] for r in frozen))
            self.assertEqual(rows[-1]['command_body'], [0.,0.,0.])
            self.assertEqual(rows[-1]['status'], 'mocap missing/stale')
            self.assertEqual(process.returncode, 0, (root/'output.log').read_text())
        node.destroy_node()
        rclpy.shutdown()
