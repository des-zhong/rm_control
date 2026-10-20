"""Compile the real agent against fake SDK headers: no hardware or network traffic."""
import json
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import tempfile
import time
import unittest


@unittest.skipUnless(shutil.which('g++'), 'g++ required')
class StopPaths(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        stub = cls.root/'fake.hpp'
        stub.write_text(r'''
#pragma once
#include <array>
#include <chrono>
#include <fstream>
#include <cstdlib>
#include <thread>
#include <atomic>
namespace unitree_go { namespace msg { namespace dds_ {
struct Stamp { int sec() const { return 0; } int nanosec() const { return 0; } };
struct Imu { std::array<float,3> rpy() const { return {0,0,0}; } };
struct SportModeState_ {
  Stamp stamp() const { return {}; } unsigned error_code() const { return 0; }
  std::array<float,3> position() const { return {0,0,0}; } Imu imu_state() const { return {}; }
};
}}}
namespace unitree { namespace robot {
struct ChannelFactory { static ChannelFactory* Instance() { static ChannelFactory f; return &f; } void Init(int,const char*) {} };
template<class T> struct ChannelSubscriber {
  std::atomic<bool> running{true}; std::thread worker;
  ChannelSubscriber(const char*) {}
  template<class F> void InitChannel(F f,int) { worker=std::thread([this,f]{ T s; while(running) { f(&s); std::this_thread::sleep_for(std::chrono::milliseconds(10)); } }); }
  ~ChannelSubscriber() { running=false; if(worker.joinable()) worker.join(); }
};
namespace as2 { struct SportClient {
  void Init() {} void SetTimeout(float) {}
  int record(const char* kind) { std::ofstream f(std::getenv("FAKE_SDK_LOG"),std::ios::app); f << kind << "\n"; return 0; }
  int Move(double,double,double) { return record("move"); }
  int StopMove() { return record("stop"); }
}; }
}}
''')
        for name in ['unitree/robot/as2/sport/sport_client.hpp',
                     'unitree/robot/channel/channel_subscriber.hpp',
                     'unitree/idl/go2/SportModeState_.hpp']:
            header = cls.root/name
            header.parent.mkdir(parents=True, exist_ok=True)
            header.write_text('#include "fake.hpp"\n')
        cls.agent = cls.root/'agent'
        source = Path(__file__).resolve().parents[1]/'remote/agent.cpp'
        subprocess.run(['g++','-std=c++17','-pthread','-I',str(cls.root),str(source),'-o',str(cls.agent)],
                       check=True, capture_output=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def launch(self):
        self.log = self.root/(self._testMethodName+'.log')
        p = subprocess.Popen([str(self.agent),'fake','--enable'], stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, env=dict(os.environ, FAKE_SDK_LOG=str(self.log)))
        def cleanup():
            if p.poll() is None:
                p.kill(); p.wait()
            if not p.stdin.closed: p.stdin.close()
            p.stdout.close()
        self.addCleanup(cleanup)
        return p

    def receive(self, p):
        ready,_,_ = select.select([p.stdout],[],[],2)
        self.assertTrue(ready)
        return json.loads(p.stdout.readline())

    def start_motion(self, p):
        state=self.receive(p)
        while not state.get('valid'):
            state=self.receive(p)
        p.stdin.write(('%d 0 -0.03 -0.06\n'%state['token']).encode())
        p.stdin.flush()
        until=time.monotonic()+1
        while time.monotonic()<until:
            self.receive(p)
            if 'move' in self.log.read_text(): return
        self.fail('No simulated Move call')

    def test_ctrl_c_in_real_local_script(self):
        # Fake ssh invokes the real agent with fake SDK; SIGINT targets the local script only.
        fake=self.root/'ssh'
        fake.write_text('#!/bin/sh\nexec '+str(self.agent)+' fake --enable\n')
        fake.chmod(0o700)
        self.log=self.root/'script.log'
        script=Path(__file__).resolve().parents[1]/'test_right.py'
        p=subprocess.Popen(['python3','-u',str(script),'--execute','--log-dir',str(self.root/'logs')],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           env=dict(os.environ, PATH=str(self.root)+os.pathsep+os.environ['PATH'],
                                    FAKE_SDK_LOG=str(self.log)))
        self.addCleanup(lambda: p.kill() if p.poll() is None else None)
        until=time.monotonic()+3
        output=b''
        while time.monotonic()<until:
            line=p.stdout.readline(); output+=line
            if b'Test started' in line: break
        self.assertIn(b'Test started',output)
        time.sleep(.15)
        p.send_signal(signal.SIGINT)
        rest,_=p.communicate(timeout=4)
        self.assertEqual(p.returncode,0,(output+rest).decode())
        self.assertIn(b'StopMove returned success',rest)
        calls=self.log.read_text().splitlines()
        self.assertIn('move',calls)
        self.assertEqual(calls[-1],'stop')

    def test_local_script_tolerates_short_telemetry_gap(self):
        fake=self.root/'ssh'
        fake.write_text("#!/usr/bin/python3\nimport subprocess,sys,time,json\n"
                        + "p=subprocess.Popen(["+repr(str(self.agent))+",'fake','--enable'],stdin=sys.stdin,stdout=subprocess.PIPE)\n"
                        + "for line in p.stdout:\n"
                        + " if json.loads(line).get('token')==3: time.sleep(.4)\n"
                        + " sys.stdout.buffer.write(line); sys.stdout.buffer.flush()\n"
                        + "raise SystemExit(p.wait())\n")
        fake.chmod(0o700)
        log=self.root/'jitter.log'
        script=Path(__file__).resolve().parents[1]/'test_right.py'
        p=subprocess.run(['python3','-u',str(script),'--execute','--seconds','1.5',
                          '--log-dir',str(self.root/'jitter-logs')], capture_output=True,text=True,timeout=5,
                         env=dict(os.environ,PATH=str(self.root)+os.pathsep+os.environ['PATH'],FAKE_SDK_LOG=str(log)))
        self.assertEqual(p.returncode,0,p.stdout+p.stderr)
        self.assertIn('State link delayed',p.stdout)
        self.assertIn('State link recovered',p.stdout)
        self.assertIn('Duration reached',p.stdout)
        self.assertIn('Remote SDK called with nonzero',p.stdout)
        self.assertEqual(log.read_text().splitlines()[-1],'stop')

    def test_watchdog_then_eof(self):
        p=self.launch(); self.start_motion(p)
        # Agent remains alive but no more commands; watchdog must stop independently.
        until=time.monotonic()+.5
        while time.monotonic()<until: self.receive(p)
        self.assertEqual(self.log.read_text().splitlines()[-1],'stop')
        p.stdin.close(); p.wait(timeout=2)
        self.assertEqual(p.returncode,0)
        self.assertEqual(self.log.read_text().splitlines()[-1],'stop')

    def test_stop_packet(self):
        p=self.launch(); self.start_motion(p)
        p.stdin.write(b'STOP\n'); p.stdin.flush()
        p.wait(timeout=2)
        rows=[json.loads(line) for line in p.stdout.read().splitlines()]
        self.assertTrue(any(r.get('kind')=='stopped' and r['rc']==0 for r in rows))
        self.assertEqual(self.log.read_text().splitlines()[-1],'stop')
