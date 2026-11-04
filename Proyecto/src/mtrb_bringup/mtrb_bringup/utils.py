"""Pure helpers shared by the launch file and the nodes (no ROS imports, easy to test)."""

import math
import os

TRUE_STRINGS = ('true', '1', 'yes')
MODES = ('map', 'mapping')


def is_true(value):
    """Return True for the launch-argument strings 'true', '1' or 'yes' (any case)."""
    return str(value).strip().lower() in TRUE_STRINGS


def absolute_topic(namespace, topic):
    """Resolve a topic name under a namespace, leaving absolute names untouched."""
    if topic.startswith('/'):
        return topic
    return f"/{namespace.strip('/')}/{topic}"


def absolute_path(path):
    """Expand '~' and make a path absolute."""
    return os.path.abspath(os.path.expanduser(path))


def yaw_to_quaternion(yaw):
    """Quaternion (x, y, z, w) for a rotation of `yaw` radians around Z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def validate_mode(mode):
    """Normalise and check the launch 'mode' argument."""
    mode = mode.strip().lower()
    if mode not in MODES:
        raise ValueError(f"Argument 'mode' must be one of {MODES} (got '{mode}')")
    return mode


def validate_namespace(namespace):
    """Strip slashes from the namespace and reject an empty one."""
    namespace = namespace.strip().strip('/')
    if not namespace:
        raise ValueError("Argument 'namespace' must not be empty")
    return namespace
