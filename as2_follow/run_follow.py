#!/usr/bin/env python3
# Connection arguments live at the beginning of this execution file.
DEFAULT_IP = '172.20.10.13'
DEFAULT_USER = 'unitree'
DEFAULT_REMOTE = '/home/unitree/as2_follow/build/as2_agent'

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from as2_follow.bridge import main

if __name__ == '__main__':
    main(default_ip=DEFAULT_IP, default_user=DEFAULT_USER, default_remote=DEFAULT_REMOTE)
