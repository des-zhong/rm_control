"""SSH transport using the fixed robot password requested by the operator."""
import contextlib
import os
from pathlib import Path
import tempfile


@contextlib.contextmanager
def ssh_environment():
    env = os.environ.copy()
    password = '123'
    with tempfile.TemporaryDirectory(prefix='as2-askpass-') as directory:
        helper = Path(directory) / 'askpass'
        helper.write_text('#!/bin/sh\nprintf "%s\\n" "$AS2_SSH_PASSWORD"\n')
        helper.chmod(0o700)
        env.update(SSH_ASKPASS=str(helper), SSH_ASKPASS_REQUIRE='force',
                   DISPLAY=env.get('DISPLAY', ':0'), AS2_SSH_PASSWORD=password)
        yield env
        env.pop('AS2_SSH_PASSWORD', None)


def ssh_options():
    return ['-T', '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=1',
            '-o', 'ServerAliveCountMax=2', '-o', 'StrictHostKeyChecking=yes']
