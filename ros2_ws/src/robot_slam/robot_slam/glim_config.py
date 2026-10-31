"""Assemble a GLIM config directory from robot_slam's config/glim/.

GLIM always reads <config_path>/config.json and resolves every other file
relative to that directory, so a GPU/CPU switch can't be a parameter. This
builds a run directory of symlinks to config/glim/*.json plus a config.json
that points at config_gpu.json or config_cpu.json.

  ros2 run robot_slam glim_config --mode gpu       # prints the directory
"""

import argparse
import os

from ament_index_python.packages import get_package_share_directory

MODES = ('gpu', 'cpu')


def assemble(mode, out_dir, source_dir=None):
    if mode not in MODES:
        raise ValueError(f'GLIM config mode must be one of {MODES}, got {mode!r}')
    if source_dir is None:
        source_dir = os.path.join(get_package_share_directory('robot_slam'), 'config', 'glim')

    os.makedirs(out_dir, exist_ok=True)
    for name in os.listdir(out_dir):
        path = os.path.join(out_dir, name)
        if os.path.islink(path):
            os.unlink(path)

    for name in os.listdir(source_dir):
        if name.endswith('.json'):
            os.symlink(os.path.join(source_dir, name), os.path.join(out_dir, name))
    os.symlink(os.path.join(source_dir, f'config_{mode}.json'), os.path.join(out_dir, 'config.json'))
    return out_dir


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--mode', choices=MODES, default='gpu')
    parser.add_argument('--out', help='output directory (default: /tmp/rover/glim_config_<mode>)')
    args = parser.parse_args()
    print(assemble(args.mode, args.out or f'/tmp/rover/glim_config_{args.mode}'))


if __name__ == '__main__':
    main()
