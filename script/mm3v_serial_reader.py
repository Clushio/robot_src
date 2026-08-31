#!/usr/bin/env python3
"""Compatibility entry point for the canonical robot_r MM3V node."""

import subprocess
import sys


def main():
    return subprocess.call([
        'ros2', 'run', 'robot_r', 'mm3v_serial_reader.py', *sys.argv[1:]
    ])


if __name__ == '__main__':
    sys.exit(main())
