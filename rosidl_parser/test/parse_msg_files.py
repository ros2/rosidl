#!/usr/bin/env python3

import argparse
import os
import sys

import rosidl_parser


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Parse all recursively found .msg files.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        'paths',
        nargs='+',
        help='The base paths to search for .msg files')
    args = parser.parse_args(argv)

    files = get_files(args.paths)

    for filename in files:
        pkg_name = os.path.basename(os.path.dirname(os.path.dirname(filename)))
        try:
            rosidl_parser.parse_message_file(pkg_name, filename)
            print(pkg_name, filename)
        except Exception as e:
            print(' ', pkg_name, filename, str(e))
            raise

    return 0


def get_files(paths):
    files = []
    for path in paths:
        if os.path.isdir(path):
            for dirpath, dirnames, filenames in os.walk(path):
                # ignore folder starting with . or _
                dirnames[:] = [d for d in dirnames if d[0] not in ['.', '_']]
                dirnames.sort()

                # select files by extension
                for filename in sorted(filenames):
                    if filename.endswith('.msg'):
                        files.append(os.path.join(dirpath, filename))
        if os.path.isfile(path):
            files.append(path)
    return files


if __name__ == '__main__':
    sys.exit(main())
