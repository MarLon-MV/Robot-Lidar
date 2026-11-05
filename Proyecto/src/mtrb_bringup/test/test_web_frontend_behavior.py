"""
Behaviour tests of web/app.js, run with Node (skipped when `node` is not installed).

Each script in test/web_js/ loads app.js in a `vm` context with a fake DOM, fetch, WebSocket
and clock (no network, no real timers, only Node built-in modules) and exits with 0 after
printing its success mark. They cover PARAR, teleop release, seq, the probe after a WS close,
maps, goal/dock/initial pose, 429/1008 and the CSP.
"""

import os
from pathlib import Path
import shutil
import subprocess

import pytest

TEST_DIR = Path(__file__).resolve().parent
JS_DIR = TEST_DIR / 'web_js'
WEB_DIR = TEST_DIR.parent / 'web'
TIMEOUT_S = 30

# script name -> mark printed on success
SCRIPTS = {
    'cancel_timeout_and_nav_warning.test.js': 'TIMEOUT+FAILED OK',
    'goals_dock_and_initial_pose.test.js': 'ALL OK',
    'maps_and_status_panel.test.js': 't18 OK',
    'offline_websocket_drop.test.js': 'OFFLINE OK',
    'teleop_and_stop_button.test.js': 't17 OK',
    'ws_1008_429_and_csp.test.js': 't19 OK',
}

NODE = shutil.which('node')
needs_node = pytest.mark.skipif(
    NODE is None, reason='node is not in PATH: the web behaviour tests need Node.js to run')


def _run_node(*args):
    env = {'PATH': os.environ.get('PATH', '')}
    return subprocess.run(
        [NODE, *map(str, args)], cwd=str(TEST_DIR), env=env, capture_output=True, text=True,
        timeout=TIMEOUT_S, check=False, shell=False)


@needs_node
@pytest.mark.parametrize('script,mark', sorted(SCRIPTS.items()))
def test_frontend_behavior_script_passes(script, mark):
    result = _run_node(JS_DIR / script, WEB_DIR)

    assert result.returncode == 0, result.stdout + result.stderr
    assert mark in result.stdout


def test_every_js_script_is_registered():
    on_disk = {p.name for p in JS_DIR.glob('*.js')}

    assert on_disk == set(SCRIPTS), (
        'scripts in test/web_js/ and SCRIPTS differ: ' + str(on_disk ^ set(SCRIPTS)))


@needs_node
def test_app_js_passes_node_check():
    result = _run_node('--check', WEB_DIR / 'app.js')

    assert result.returncode == 0, result.stderr


def test_web_js_is_not_installed_as_data_files():
    setup_py = (TEST_DIR.parent / 'setup.py').read_text(encoding='utf-8')

    assert 'web_js' not in setup_py
    assert "glob('test" not in setup_py
