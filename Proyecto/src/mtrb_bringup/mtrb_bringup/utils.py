"""Pure helpers shared by the launch file and the nodes (no ROS imports, easy to test)."""

import math
import os

TRUE_STRINGS = ('true', '1', 'yes')
MODES = ('map', 'mapping')
WEB_PORT_MIN = 1024  # unprivileged ports only
WEB_PORT_MAX = 65535


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


def validate_web_port(port):
    """Parse the launch 'web_port' argument and check it is in 1024-65535. Returns an int."""
    try:
        value = int(str(port).strip())
    except ValueError:
        raise ValueError(f"Argument 'web_port' must be an integer (got '{port}')") from None
    if not WEB_PORT_MIN <= value <= WEB_PORT_MAX:
        raise ValueError(
            f"Argument 'web_port' must be in {WEB_PORT_MIN}-{WEB_PORT_MAX} (got {value})")
    return value
