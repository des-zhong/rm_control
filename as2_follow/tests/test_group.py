import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest


class GroupTests(unittest.TestCase):
    def exercise(self, early_failure):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            child=root/'child.py'
            child.write_text('''import signal,sys,time
from pathlib import Path
mark=Path(sys.argv[1])
signal.signal(signal.SIGINT,lambda *_:(mark.write_text('stopped'),sys.exit(0)))
mark.write_text('ready')
if len(sys.argv)>2: sys.exit(7)
while True: time.sleep(.05)
''')
            commands={side:[sys.executable,str(child),str(root/side)] for side in ('lower','upper')}
            if early_failure: commands['lower'].append('fail')
            code='from as2_follow.group import supervise; import os; raise SystemExit(supervise('+repr(commands)+',os.environ.copy()))'
            env=dict(os.environ)
            env['PYTHONPATH']=str(Path(__file__).resolve().parents[1])+os.pathsep+env.get('PYTHONPATH','')
            process=subprocess.Popen([sys.executable,'-c',code],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=env)
            try:
                deadline=time.monotonic()+4
                while time.monotonic()<deadline and not all((root/side).exists() for side in commands):
                    time.sleep(.02)
                self.assertTrue(all((root/side).exists() for side in commands))
                time.sleep(.2)
                self.assertIsNone(process.poll())
                self.assertEqual((root/'upper').read_text(),'ready')
                process.send_signal(signal.SIGINT)
                output,_=process.communicate(timeout=5)
                self.assertEqual(process.returncode,1 if early_failure else 0,output.decode())
                self.assertEqual((root/'upper').read_text(),'stopped')
                if not early_failure:self.assertEqual((root/'lower').read_text(),'stopped')
            finally:
                if process.poll() is None:process.kill();process.wait()
                process.stdout.close()

    def test_ctrl_c_stops_both(self):
        self.exercise(False)

    def test_one_failure_keeps_other_running(self):
        self.exercise(True)
