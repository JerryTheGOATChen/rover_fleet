"""Turning the single-robot parameter files into per-robot ones (used by fleet.launch.py).

The fleet shares one TF tree, so robot1's frames are robot1/map,
robot1/odom, robot1/body_link... and its absolute topics are /robot1/scan
etc. Rather than keep a second copy of every config, the launch file
rewrites the single-robot files:

    global_frame: map      ->  global_frame: robot1/map
    scan_topic: /scan      ->  scan_topic: /robot1/scan

and nests everything under the namespace (`robot1: bt_navigator: ...`), which
is how a ROS 2 params file addresses namespaced nodes.
"""

import os
import tempfile

import yaml

FRAMES = ('map', 'odom', 'body_link', 'lidar_link')
ABSOLUTE_TOPICS = ('/scan', '/odom', '/map')


def rewrite_for_robot(tree, ns: str):
    """Return a copy of a parsed params file with frame names and absolute topics prefixed."""
    if isinstance(tree, dict):
        return {k: rewrite_for_robot(v, ns) for k, v in tree.items()}
    if isinstance(tree, list):
        return [rewrite_for_robot(v, ns) for v in tree]
    if isinstance(tree, str):
        if tree in FRAMES:
            return f'{ns}/{tree}'
        if tree in ABSOLUTE_TOPICS:
            return f'/{ns}{tree}'
    return tree


class _NoAliasDumper(yaml.SafeDumper):
    """ROS 2's params parser rejects YAML aliases, so write repeated values out in full."""

    def ignore_aliases(self, data):
        return True


def _merge(base, extra):
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value
    return base


def namespaced_params(path: str, ns: str, overrides: dict = None) -> str:
    """Write robot `ns`'s version of the params file at `path`; return the new file's path.

    `overrides` (same nesting as the file, e.g. {'bt_navigator': {'ros__parameters':
    {...}}}) is merged in after the rewrite.
    """
    with open(path) as f:
        tree = yaml.safe_load(f)
    fd, out = tempfile.mkstemp(prefix=f'{ns}_', suffix='_' + os.path.basename(path))
    with os.fdopen(fd, 'w') as f:
        yaml.dump({ns: _merge(rewrite_for_robot(tree, ns), overrides or {})}, f,
                  Dumper=_NoAliasDumper, sort_keys=False)
    return out
