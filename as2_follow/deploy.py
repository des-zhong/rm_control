#!/usr/bin/env python3
# Destination is configurable here or via --ip, independently for each dog.
DEFAULT_IP = '172.20.10.13'
DEFAULT_USER = 'unitree'
SDK_REVISION = 'd1abf001220d3e5f59e00a0c4aaa595e00089a4a'

import argparse
from pathlib import Path
import shlex
import subprocess
import tarfile
import tempfile

from as2_follow.transport import ssh_environment, ssh_options


def main():
    p = argparse.ArgumentParser(description='Copy/build isolated agent. Does not start robot control.')
    p.add_argument('--ip', default=DEFAULT_IP)
    p.add_argument('--user', default=DEFAULT_USER)
    p.add_argument('--sdk-dir', type=Path)
    p.add_argument('--agent-only', action='store_true', help='Reuse SDK already deployed on dog')
    args = p.parse_args()
    sdk = None
    if not args.agent_only:
        if args.sdk_dir is None:
            p.error('--sdk-dir is required on first deployment')
        sdk = args.sdk_dir.resolve()
        revision = subprocess.check_output(['git', '-C', str(sdk), 'rev-parse', 'HEAD'], text=True).strip()
        if revision != SDK_REVISION:
            p.error('SDK must be pinned to ' + SDK_REVISION)
        if subprocess.check_output(['git', '-C', str(sdk), 'status', '--porcelain'], text=True).strip():
            p.error('SDK checkout must be clean')
    destination = args.user + '@' + args.ip
    if args.ip.startswith('-') or args.user.startswith('-'):
        p.error('Invalid SSH destination')
    root = Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory(prefix='as2-deploy-') as temp, ssh_environment() as env:
        archive = Path(temp) / 'as2_follow.tar.gz'
        def exclude_git(info):
            parts = Path(info.name).parts
            return None if '.git' in parts or 'x86_64' in parts else info
        with tarfile.open(archive, 'w:gz') as tar:
            if sdk is not None:
                tar.add(sdk, arcname='sdk', filter=exclude_git)
            tar.add(root / 'remote', arcname='remote')
        # stdin avoids scp destination/path expansion; fixed destination under robot user's HOME.
        command = ('set -eu\n'
                   'mkdir -p "$HOME/as2_follow"\n'
                   'tar -xzf - -C "$HOME/as2_follow"\n'
                   'cmake -S "$HOME/as2_follow/remote" -B "$HOME/as2_follow/build" '
                   '-DUNITREE_SDK_DIR="$HOME/as2_follow/sdk"\n'
                   'cmake --build "$HOME/as2_follow/build" -j2\n')
        with archive.open('rb') as source:
            subprocess.run(['ssh', *ssh_options(), destination, 'sh -c ' + shlex.quote(command)],
                           stdin=source, env=env, start_new_session=True, check=True)
    print('Deployed and built. No control process was started.')


if __name__ == '__main__':
    main()
