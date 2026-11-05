"""Audit: every `raise ... from <cause>` in the web bridge is a reviewed one (no secrets)."""

import ast
import collections
import pathlib

import pytest

PACKAGE = pathlib.Path(__file__).resolve().parent.parent / 'mtrb_bringup'
AUDITED_FILES = ('web_server.py', 'mtrb_web_bridge.py', 'web_utils.py')

# SECURITY: `raise X from err` keeps `err` in X.__cause__ and its args/repr can carry the
# token bytes or what the client sent (body, map name, goal). A new `from err` on a path
# that touches the token or client data must be `from None`. This table lists the ones
# REVIEWED as safe: they only carry ROS data or system errors with no client content.
# Key: (file, enclosing function, public exception, number of sites). If this test fails
# you added (or removed) a `from <cause>`: review what the cause can carry, then update
# the table with a reason.
REVIEWED = {
    # OSError of os.open/os.read on the token file: errno and path, never the content;
    # the message only has the exception class name.
    ('web_server.py', 'read_token_file', 'TokenError'): 2,
    # OSError of the map_dir (listing, creating, quotas, backups): paths of the operator.
    ('web_server.py', 'MapSaver.names', 'ApiError'): 1,
    ('web_server.py', 'MapSaver._prepare', 'ApiError'): 1,
    ('web_server.py', 'MapSaver._check_quotas', 'ApiError'): 1,
    ('web_server.py', 'MapSaver._backup', 'ApiError'): 1,
    # slam_toolbox save_map: unavailable / timeout / failed (ROS data; the validated map
    # name is already a safe [A-Za-z0-9_-] string at this point).
    ('web_server.py', 'MapSaver._save_locked', 'ApiError'): 4,
    # NavBusyError / NavUnavailableError raised by our own NavPort with fixed texts.
    ('web_server.py', 'create_app.run_nav', 'ApiError'): 2,
    # Port from the YAML (operator configuration, not a secret nor client data).
    ('mtrb_web_bridge.py', 'MtrbWebBridge.__init__', 'ConfigError'): 1,
    # TokenError from load_token: its messages are fixed texts (see read_token_file).
    ('mtrb_web_bridge.py', 'MtrbWebBridge._load_token', 'ConfigError'): 1,
    # rclpy service call failure: ROS data.
    ('mtrb_web_bridge.py', 'MtrbWebBridge._call_service', 'SlamUnavailableError'): 1,
    # rclpy action client failure when sending a goal: ROS data.
    ('mtrb_web_bridge.py', 'MtrbWebBridge.send_goal', 'NavUnavailableError'): 1,
    # bind / listen error (address in use, permission): host and port of the operator.
    ('mtrb_web_bridge.py', 'MtrbWebBridge.start_server', 'ConfigError'): 1,
    # web_utils: conversions of ROS messages (map, scan, path) and of the stored map info
    # to ValueError. The cause is a TypeError/ValueError about ROS values.
    ('web_utils.py', 'occupancy_to_bytes', 'ValueError'): 2,
    ('web_utils.py', 'downsample_scan', 'ValueError'): 1,
    ('web_utils.py', 'scan_to_map_points', 'ValueError'): 2,
    ('web_utils.py', 'simplify_path', 'ValueError'): 1,
    # validate_goal: map info comes from the /map topic, not from the client; the client's
    # x/y/yaw are checked before and never reach these causes.
    ('web_utils.py', 'validate_goal', 'ValueError'): 3,
}


def _chained_raises(file_name, base=PACKAGE):
    """Return {(file, function, exception): [line, ...]} of `raise ... from <cause>`."""
    tree = ast.parse((base / file_name).read_text(encoding='utf-8'))
    found = collections.defaultdict(list)

    def visit(node, scope):
        for child in ast.iter_child_nodes(node):
            inner = scope
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                inner = scope + [child.name]
            if isinstance(child, ast.Raise) and child.cause is not None:
                is_none = isinstance(child.cause, ast.Constant) and child.cause.value is None
                if not is_none:
                    exc = child.exc.func if isinstance(child.exc, ast.Call) else child.exc
                    found[(file_name, '.'.join(scope), ast.unparse(exc))].append(child.lineno)
            visit(child, inner)

    visit(tree, [])
    return found


def test_every_raise_from_a_cause_is_in_the_reviewed_list():
    actual = {}
    for file_name in AUDITED_FILES:
        for key, lines in _chained_raises(file_name).items():
            actual[key] = len(lines)
    unreviewed = {key: n for key, n in actual.items() if REVIEWED.get(key) != n}
    assert unreviewed == {}, (
        'New or changed `raise ... from <cause>`: if it touches the token or client data '
        'use `from None`; otherwise review it and add it to REVIEWED')


def test_the_reviewed_list_has_no_stale_entries():
    actual = {}
    for file_name in AUDITED_FILES:
        for key, lines in _chained_raises(file_name).items():
            actual[key] = len(lines)
    assert {key for key in REVIEWED if key not in actual} == set()


@pytest.mark.parametrize('file_name', AUDITED_FILES)
def test_the_audited_files_exist(file_name):
    assert (PACKAGE / file_name).is_file()


def test_the_scanner_detects_a_chained_raise_and_ignores_from_none(tmp_path):
    (tmp_path / 'sample.py').write_text(
        'def f():\n    try:\n        pass\n    except OSError as err:\n'
        '        raise ValueError("x") from err\n'
        'def g():\n    try:\n        pass\n    except OSError:\n'
        '        raise ValueError("x") from None\n', encoding='utf-8')
    assert dict(_chained_raises('sample.py', tmp_path)) == {
        ('sample.py', 'f', 'ValueError'): [5]}
