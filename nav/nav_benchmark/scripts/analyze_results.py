#!/usr/bin/env python3

import argparse
import os
import sys

from nav_benchmark.analysis import generate_report


def session_directories(arguments):
    if arguments.sessions:
        return [os.path.abspath(os.path.expanduser(path)) for path in arguments.sessions]
    root = os.path.abspath(os.path.expanduser(arguments.results_root))
    candidates = [
        os.path.join(root, name) for name in os.listdir(root)
        if os.path.isdir(os.path.join(root, name))
        and os.path.isfile(os.path.join(root, name, 'session.json'))
    ]
    if not candidates:
        raise RuntimeError('no benchmark sessions found in {}'.format(root))
    return [max(candidates, key=os.path.getmtime)]


def main():
    parser = argparse.ArgumentParser(
        description='Summarize one or more nav_benchmark sessions.'
    )
    parser.add_argument('sessions', nargs='*', help='Session directories')
    parser.add_argument(
        '--results-root', default='~/maps/nav_benchmark_results',
        help='Uses the newest session when no session directory is supplied',
    )
    parser.add_argument('--output-dir', help='Output directory for combined results')
    arguments = parser.parse_args()
    try:
        directories = session_directories(arguments)
        json_path, text_path = generate_report(
            directories, output_dir=arguments.output_dir
        )
    except (OSError, RuntimeError, ValueError) as error:
        parser.error(str(error))
    print(json_path)
    print(text_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
