#!/usr/bin/env python3
# Connection arguments for the two independent observers.
LOWER_IP = '172.20.10.13'
UPPER_IP = '172.20.10.14'
SSH_USER = 'unitree'

from as2_follow.group import main

if __name__ == '__main__':
    raise SystemExit(main(LOWER_IP, UPPER_IP, SSH_USER))
