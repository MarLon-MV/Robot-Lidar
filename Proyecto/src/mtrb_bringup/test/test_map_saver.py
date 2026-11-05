"""Tests of the phase 5 map saving: MapSaver, POST /api/map/save and GET /api/maps (no ROS).

The slam_toolbox side is a fake MapPort that writes real files into a real temporary
directory, so the backup / restore / confinement logic is exercised for real. Concurrency is
driven with threading.Event (no sleeps) and the 30 s pause with an injected clock.
"""

import ast
import collections
import json
import os
import threading
from unittest import mock

import pytest

pytest.importorskip('fastapi')
pytest.importorskip('httpx')

from fastapi.testclient import TestClient  # noqa: E402, I100
from mtrb_bringup import web_server  # noqa: E402, I100
from mtrb_bringup.web_server import (  # noqa: E402, I100
    ApiError, create_app, MAP_SAVE_COOLDOWN_S, MAP_SAVE_TIMEOUT_S, MAP_SERIALIZE_TIMEOUT_S,
    MapPort, MapSaver, NavPort, SlamTimeoutError, SlamUnavailableError, StateStore)
from mtrb_bringup.web_utils import MAP_PATH_PATTERN, valid_map_name  # noqa: E402, I100

TOKEN = 'secret-token-123'
AUTH = {'Authorization': f'Bearer {TOKEN}'}
SECRET_TEXT = 'LEAK-internal-detail'
WAIT_S = 10.0  # upper bound for a thread to reach a point (never reached when all is well)


@pytest.fixture
def map_dir(tmp_path):
    """Return a real, empty map directory whose path slam_toolbox accepts."""
    real = os.path.realpath(str(tmp_path))
    if MAP_PATH_PATTERN.fullmatch(real) is None:
        pytest.skip(f'tmp_path has characters outside [A-Za-z0-9_./-]: {real}')
    path = os.path.join(real, 'maps')
    os.mkdir(path)
    return path


def _write(path, content=b'new'):
    with open(path, 'wb') as handle:
        handle.write(content)


def _read(path):
    with open(path, 'rb') as handle:
        return handle.read()


def _files(directory):
    return sorted(os.listdir(directory))


class FakePort(MapPort):
    """MapPort double: writes real files by default and records every call."""

    def __init__(self):
        self.is_ready = True
        self.calls = []              # ('save' | 'serialize', base_path, timeout_s)
        self.save_hook = None        # callable(base) run instead of the default writer
        self.serialize_hook = None
        self.save_result = True
        self.serialize_result = True

    def ready(self):
        self.calls.append(('ready',))
        return self.is_ready

    def save_map(self, base_path, timeout_s):
        self.calls.append(('save', base_path, timeout_s))
        if self.save_hook is not None:
            return self.save_hook(base_path)
        _write(base_path + '.yaml', b'new-yaml')
        _write(base_path + '.pgm', b'new-pgm')
        return self.save_result

    def serialize_posegraph(self, base_path, timeout_s):
        self.calls.append(('serialize', base_path, timeout_s))
        if self.serialize_hook is not None:
            return self.serialize_hook(base_path)
        _write(base_path + '.posegraph', b'new-pg')
        _write(base_path + '.data', b'new-data')
        return self.serialize_result

    def saves(self):
        return [c for c in self.calls if c[0] == 'save']


class Clock:
    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now


def _saver(directory, port=None, clock=None, min_free_mb=0, min_interval_s=0.0):
    """Build a MapSaver; by default no disk quota and no rate limit (tests opt in)."""
    port = FakePort() if port is None else port
    events = []
    saver = MapSaver(directory, port, clock=clock or Clock(),
                     on_event=lambda level, text: events.append((level, text)),
                     min_free_mb=min_free_mb, min_interval_s=min_interval_s)
    return saver, port, events


def _old_map(directory, name='lab', exts=('.yaml', '.pgm', '.posegraph', '.data')):
    for ext in exts:
        _write(os.path.join(directory, name + ext), b'old' + ext.encode())


def _error(callable_, *args):
    with pytest.raises(ApiError) as caught:
        callable_(*args)
    return caught.value


# --- success and existing maps -------------------------------------------------------------

def test_ca_save_success_returns_ok_name_and_posegraph_true(map_dir):
    saver, port, events = _saver(map_dir)
    assert saver.save('lab', False) == {'ok': True, 'name': 'lab', 'posegraph': True}
    assert _files(map_dir) == ['lab.data', 'lab.pgm', 'lab.posegraph', 'lab.yaml']
    assert events[-1][0] == 'info'


def test_save_sends_slam_toolbox_one_absolute_confined_base_path(map_dir):
    saver, port, _ = _saver(map_dir)
    saver.save('lab', False)
    [(_, base, _)] = port.saves()
    assert base == os.path.join(map_dir, 'lab')
    assert base.startswith('/')
    assert MAP_PATH_PATTERN.fullmatch(base) is not None
    serialized = [c for c in port.calls if c[0] == 'serialize']
    assert [c[1] for c in serialized] == [base]


def test_save_uses_bounded_positive_timeouts_for_slam_toolbox(map_dir):
    saver, port, _ = _saver(map_dir)
    saver.save('lab', False)
    timeouts = {c[0]: c[2] for c in port.calls if c[0] in ('save', 'serialize')}
    assert timeouts == {'save': MAP_SAVE_TIMEOUT_S, 'serialize': MAP_SERIALIZE_TIMEOUT_S}
    assert 0 < MAP_SAVE_TIMEOUT_S <= 60 and 0 < MAP_SERIALIZE_TIMEOUT_S <= 60


def test_save_creates_a_missing_map_directory(tmp_path):
    real = os.path.realpath(str(tmp_path))
    target = os.path.join(real, 'new', 'maps')
    if MAP_PATH_PATTERN.fullmatch(target) is None:
        pytest.skip('tmp_path has unusual characters')
    saver, _, _ = _saver(target)
    assert saver.save('lab', False)['ok'] is True
    assert os.path.isfile(os.path.join(target, 'lab.yaml'))


def test_save_with_png_image_is_accepted(map_dir):
    port = FakePort()

    def writer(base):
        _write(base + '.yaml')
        _write(base + '.png')
        return True

    port.save_hook = writer
    saver, _, _ = _saver(map_dir, port)
    assert saver.save('lab', False)['ok'] is True


@pytest.mark.parametrize('ext', ['.yaml', '.pgm', '.png', '.posegraph', '.data'])
def test_ca_save_without_overwrite_answers_409_map_exists_for_any_single_file(map_dir, ext):
    # Also covers a lone stray `.data` file: any file of the map blocks a plain save.
    _write(os.path.join(map_dir, 'lab' + ext), b'precious')
    saver, port, _ = _saver(map_dir)
    err = _error(saver.save, 'lab', False)
    assert (err.status_code, err.code) == (409, 'map_exists')
    assert _files(map_dir) == ['lab' + ext]
    assert _read(os.path.join(map_dir, 'lab' + ext)) == b'precious'
    assert port.saves() == []


def test_other_maps_do_not_block_a_save(map_dir):
    _old_map(map_dir, 'other')
    saver, _, _ = _saver(map_dir)
    assert saver.save('lab', False)['ok'] is True
    assert _read(os.path.join(map_dir, 'other.yaml')) == b'old.yaml'


# --- overwrite ---------------------------------------------------------------------------

def test_ca_overwrite_success_replaces_the_files_and_deletes_the_backups(map_dir):
    _old_map(map_dir)
    saver, _, _ = _saver(map_dir)
    assert saver.save('lab', True) == {'ok': True, 'name': 'lab', 'posegraph': True}
    assert _files(map_dir) == ['lab.data', 'lab.pgm', 'lab.posegraph', 'lab.yaml']
    assert _read(os.path.join(map_dir, 'lab.yaml')) == b'new-yaml'
    assert not [f for f in _files(map_dir) if f.endswith('.bak')]


def test_overwrite_true_on_a_new_name_just_saves(map_dir):
    saver, _, _ = _saver(map_dir)
    assert saver.save('fresh', True)['ok'] is True


def test_overwrite_moves_the_old_files_aside_before_slam_toolbox_writes(map_dir):
    _old_map(map_dir)
    port = FakePort()
    seen = {}

    def writer(base):
        seen['files'] = _files(map_dir)
        seen['bak'] = _read(base + '.yaml.bak')
        _write(base + '.yaml')
        _write(base + '.pgm')
        return True

    port.save_hook = writer
    saver, _, _ = _saver(map_dir, port)
    saver.save('lab', True)
    assert 'lab.yaml' not in seen['files'] and 'lab.yaml.bak' in seen['files']
    assert seen['bak'] == b'old.yaml'


@pytest.mark.parametrize('how', ['returns_false', 'raises', 'no_files', 'no_image'])
def test_ca_overwrite_failure_removes_new_files_and_restores_the_backups(map_dir, how):
    _old_map(map_dir)
    before = {f: _read(os.path.join(map_dir, f)) for f in _files(map_dir)}
    port = FakePort()

    def writer(base):
        _write(base + '.yaml', b'half-written')
        _write(base + '.png', b'half-written')
        if how == 'raises':
            raise RuntimeError(SECRET_TEXT)
        if how == 'no_files':
            os.remove(base + '.yaml')
            os.remove(base + '.png')
        if how == 'no_image':
            os.remove(base + '.png')
        return how != 'returns_false'

    port.save_hook = writer
    saver, _, events = _saver(map_dir, port)
    err = _error(saver.save, 'lab', True)
    assert (err.status_code, err.code) == (500, 'save_failed')
    assert {f: _read(os.path.join(map_dir, f)) for f in _files(map_dir)} == before
    assert any(level == 'error' for level, _ in events)


def test_overwrite_failure_because_slam_toolbox_vanished_restores_with_503(map_dir):
    _old_map(map_dir)
    port = FakePort()

    def writer(base):
        raise SlamUnavailableError('gone')

    port.save_hook = writer
    saver, _, _ = _saver(map_dir, port)
    err = _error(saver.save, 'lab', True)
    assert (err.status_code, err.code) == (503, 'slam_unavailable')
    assert _files(map_dir) == ['lab.data', 'lab.pgm', 'lab.posegraph', 'lab.yaml']
    assert _read(os.path.join(map_dir, 'lab.pgm')) == b'old.pgm'


def test_failed_plain_save_leaves_no_partial_files(map_dir):
    port = FakePort()
    port.save_result = False   # default writer leaves a yaml + pgm and reports an error
    saver, _, _ = _saver(map_dir, port)
    err = _error(saver.save, 'lab', False)
    assert (err.status_code, err.code) == (500, 'save_failed')
    assert _files(map_dir) == []


def test_save_failed_when_slam_toolbox_says_ok_but_writes_nothing(map_dir):
    port = FakePort()
    port.save_hook = lambda base: True
    saver, _, _ = _saver(map_dir, port)
    err = _error(saver.save, 'lab', False)
    assert (err.status_code, err.code) == (500, 'save_failed')


def test_save_failed_when_the_written_yaml_is_empty(map_dir):
    port = FakePort()

    def writer(base):
        _write(base + '.yaml', b'')
        _write(base + '.pgm', b'img')
        return True

    port.save_hook = writer
    saver, _, _ = _saver(map_dir, port)
    assert _error(saver.save, 'lab', False).code == 'save_failed'
    assert _files(map_dir) == []


# --- timeout and pause -------------------------------------------------------------------

def _timeout_port(partial=True):
    port = FakePort()

    def writer(base):
        if partial:
            _write(base + '.yaml', b'partial')
        raise SlamTimeoutError('late')

    port.save_hook = writer
    return port


def test_ca_timeout_answers_504_and_touches_no_file_leaving_the_backups(map_dir):
    _old_map(map_dir)
    saver, _, events = _saver(map_dir, _timeout_port())
    err = _error(saver.save, 'lab', True)
    assert (err.status_code, err.code) == (504, 'save_timeout')
    # slam_toolbox may still be writing: neither deleted nor restored, backups stay for a
    # manual recovery; the partial file written by the (fake) late writer is left alone.
    assert _files(map_dir) == ['lab.data.bak', 'lab.pgm.bak', 'lab.posegraph.bak',
                               'lab.yaml', 'lab.yaml.bak']
    assert _read(os.path.join(map_dir, 'lab.yaml')) == b'partial'
    assert _read(os.path.join(map_dir, 'lab.yaml.bak')) == b'old.yaml'
    assert events[-1][0] == 'error'


def test_ca_after_a_timeout_saves_are_paused_for_30_seconds_with_the_injected_clock(map_dir):
    clock = Clock(1000.0)
    port = _timeout_port(partial=False)
    saver, _, _ = _saver(map_dir, port, clock)
    assert _error(saver.save, 'lab', False).code == 'save_timeout'
    saves_before = len(port.saves())
    port.save_hook = None
    for elapsed in (0.0, 1.0, MAP_SAVE_COOLDOWN_S - 1.0):
        clock.now = 1000.0 + elapsed
        err = _error(saver.save, 'other', False)
        assert (err.status_code, err.code) == (409, 'save_in_progress')
    assert len(port.saves()) == saves_before      # slam_toolbox was not asked again
    clock.now = 1000.0 + MAP_SAVE_COOLDOWN_S + 0.5
    assert saver.save('other', False)['ok'] is True


def test_the_pause_is_30_seconds():
    assert MAP_SAVE_COOLDOWN_S == 30.0


def test_a_posegraph_timeout_also_starts_the_pause_but_the_save_succeeds(map_dir):
    clock = Clock(50.0)
    port = FakePort()

    def slow(base):
        raise SlamTimeoutError('slow')

    port.serialize_hook = slow
    saver, _, events = _saver(map_dir, port, clock)
    assert saver.save('lab', False) == {'ok': True, 'name': 'lab', 'posegraph': False}
    assert _error(saver.save, 'other', False).code == 'save_in_progress'
    clock.now += MAP_SAVE_COOLDOWN_S + 1
    assert saver.save('other', False)['ok'] is True
    assert any(level == 'warning' for level, _ in events)


def test_a_plain_failure_does_not_start_the_pause(map_dir):
    port = FakePort()
    port.save_result = False
    saver, _, _ = _saver(map_dir, port)
    assert _error(saver.save, 'lab', False).code == 'save_failed'
    port.save_result = True
    assert saver.save('lab', False)['ok'] is True


# --- concurrency -------------------------------------------------------------------------

def test_ca_a_concurrent_save_answers_409_save_in_progress_and_the_lock_is_released(map_dir):
    entered, release = threading.Event(), threading.Event()
    port = FakePort()

    def blocking(base):
        entered.set()
        assert release.wait(WAIT_S)
        _write(base + '.yaml')
        _write(base + '.pgm')
        return True

    port.save_hook = blocking
    saver, _, _ = _saver(map_dir, port)
    result = {}
    worker = threading.Thread(target=lambda: result.update(saver.save('first', False)))
    worker.start()
    try:
        assert entered.wait(WAIT_S)
        for name in ('first', 'second'):
            err = _error(saver.save, name, False)
            assert (err.status_code, err.code) == (409, 'save_in_progress')
        assert len(port.saves()) == 1
    finally:
        release.set()
        worker.join(WAIT_S)
    assert not worker.is_alive()
    assert result == {'ok': True, 'name': 'first', 'posegraph': True}
    port.save_hook = None
    assert saver.save('second', False)['ok'] is True


def test_the_lock_is_released_after_every_kind_of_failure(map_dir):
    saver, port, _ = _saver(map_dir)
    port.is_ready = False
    assert _error(saver.save, 'a', False).code == 'slam_unavailable'
    port.is_ready = True
    port.save_hook = lambda base: (_ for _ in ()).throw(RuntimeError(SECRET_TEXT))
    assert _error(saver.save, 'a', False).code == 'save_failed'
    port.save_hook = None
    _write(os.path.join(map_dir, 'a.yaml'))
    assert _error(saver.save, 'a', False).code == 'map_exists'
    assert saver.save('b', False)['ok'] is True


# --- other errors ------------------------------------------------------------------------

def test_ca_slam_unavailable_answers_503_and_writes_nothing(map_dir):
    saver, port, _ = _saver(map_dir)
    port.is_ready = False
    err = _error(saver.save, 'lab', False)
    assert (err.status_code, err.code) == (503, 'slam_unavailable')
    assert port.saves() == []
    assert _files(map_dir) == []


def test_slam_unavailable_wins_over_an_existing_map_so_nothing_is_moved(map_dir):
    _old_map(map_dir)
    saver, port, _ = _saver(map_dir)
    port.is_ready = False
    assert _error(saver.save, 'lab', True).code == 'slam_unavailable'
    assert _files(map_dir) == ['lab.data', 'lab.pgm', 'lab.posegraph', 'lab.yaml']


@pytest.mark.parametrize('bad_dir', [None, '', 'relative/maps', 5])
def test_ca_an_unusable_map_dir_answers_500_map_dir_invalid(bad_dir):
    saver, port, _ = _saver(bad_dir)
    err = _error(saver.save, 'lab', False)
    assert (err.status_code, err.code) == (500, 'map_dir_invalid')
    assert port.saves() == []


def test_map_dir_with_a_space_answers_500_map_dir_invalid(tmp_path):
    bad = os.path.join(os.path.realpath(str(tmp_path)), 'with space')
    saver, port, _ = _saver(bad)
    err = _error(saver.save, 'lab', False)
    assert err.code == 'map_dir_invalid'
    assert port.saves() == []
    assert 'with space' not in err.detail


def test_map_dir_that_is_a_file_answers_500_map_dir_invalid(map_dir):
    target = os.path.join(map_dir, 'afile')
    _write(target)
    saver, port, _ = _saver(target)
    assert _error(saver.save, 'lab', False).code == 'map_dir_invalid'
    assert port.saves() == []


def test_map_dir_that_is_the_filesystem_root_answers_500_map_dir_invalid():
    saver, port, _ = _saver('/')
    assert _error(saver.save, 'lab', False).code == 'map_dir_invalid'
    assert port.saves() == []


@pytest.mark.parametrize('ext', ['.yaml', '.pgm', '.data', '.yaml.bak'])
def test_a_symlink_among_the_map_files_answers_500_and_is_never_followed(map_dir, ext):
    outside = os.path.join(os.path.dirname(map_dir), 'victim')
    _write(outside, b'do-not-touch')
    os.symlink(outside, os.path.join(map_dir, 'lab' + ext))
    saver, port, _ = _saver(map_dir)
    for overwrite in (False, True):
        err = _error(saver.save, 'lab', overwrite)
        assert (err.status_code, err.code) == (500, 'map_dir_invalid')
    assert _read(outside) == b'do-not-touch'
    assert port.saves() == []


def test_a_directory_in_place_of_a_map_file_answers_500_map_dir_invalid(map_dir):
    os.mkdir(os.path.join(map_dir, 'lab.yaml'))
    saver, port, _ = _saver(map_dir)
    assert _error(saver.save, 'lab', True).code == 'map_dir_invalid'
    assert port.saves() == []


def test_an_unwritable_backup_answers_500_map_dir_invalid_and_undoes_the_renames(
        map_dir, monkeypatch):
    _old_map(map_dir)
    real_replace = os.replace
    calls = []

    def flaky(source, target):
        calls.append((source, target))
        if len(calls) == 3:
            raise OSError('disk error ' + SECRET_TEXT)
        return real_replace(source, target)

    monkeypatch.setattr(web_server.os, 'replace', flaky)
    saver, port, _ = _saver(map_dir)
    err = _error(saver.save, 'lab', True)
    assert (err.status_code, err.code) == (500, 'map_dir_invalid')
    assert SECRET_TEXT not in err.detail
    assert _files(map_dir) == ['lab.data', 'lab.pgm', 'lab.posegraph', 'lab.yaml']
    assert port.saves() == []


# --- posegraph (best effort) -------------------------------------------------------------

@pytest.mark.parametrize('how', ['false', 'only_posegraph', 'raises', 'unavailable'])
def test_ca_posegraph_failure_is_a_partial_success_with_posegraph_false(map_dir, how):
    port = FakePort()

    def writer(base):
        if how == 'raises':
            raise RuntimeError(SECRET_TEXT)
        if how == 'unavailable':
            raise SlamUnavailableError('down')
        _write(base + '.posegraph')
        if how == 'only_posegraph':
            return True
        return False

    port.serialize_hook = writer
    saver, _, events = _saver(map_dir, port)
    assert saver.save('lab', False) == {'ok': True, 'name': 'lab', 'posegraph': False}
    assert os.path.isfile(os.path.join(map_dir, 'lab.yaml'))
    assert any(level == 'warning' for level, _ in events)


def test_ca_overwrite_with_a_failed_posegraph_removes_the_old_posegraph(map_dir):
    _old_map(map_dir)
    port = FakePort()
    port.serialize_hook = lambda base: False
    saver, _, _ = _saver(map_dir, port)
    assert saver.save('lab', True) == {'ok': True, 'name': 'lab', 'posegraph': False}
    # A new map next to a stale posegraph would let `posegraph:=` load a different map.
    assert _files(map_dir) == ['lab.pgm', 'lab.yaml']
    assert _read(os.path.join(map_dir, 'lab.yaml')) == b'new-yaml'


# --- generic exceptions never reach the client -------------------------------------------

@pytest.mark.parametrize('error', [RuntimeError(SECRET_TEXT), OSError(SECRET_TEXT),
                                   ValueError(SECRET_TEXT), KeyError(SECRET_TEXT)])
def test_ca_any_exception_of_the_port_becomes_a_controlled_500_without_its_text(
        map_dir, error):
    port = FakePort()

    def boom(base):
        raise error

    port.save_hook = boom
    saver, _, events = _saver(map_dir, port)
    err = _error(saver.save, 'lab', False)
    assert (err.status_code, err.code) == (500, 'save_failed')
    assert SECRET_TEXT not in err.detail and SECRET_TEXT not in err.code
    assert _files(map_dir) == []


# --- names() / listing -------------------------------------------------------------------

def test_names_lists_sorted_complete_maps(map_dir):
    saver, _, _ = _saver(map_dir)
    saver.save('beta', False)
    saver.save('alpha', False)
    _write(os.path.join(map_dir, 'half.yaml'))
    assert saver.names() == ['alpha', 'beta']


def test_names_of_a_missing_directory_is_empty(tmp_path):
    saver, _, _ = _saver(os.path.join(os.path.realpath(str(tmp_path)), 'missing'))
    assert saver.names() == []


def test_names_of_a_relative_existing_directory_is_500_map_dir_invalid(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    os.mkdir('relative')
    saver, _, _ = _saver('relative')
    err = _error(saver.names)
    assert (err.status_code, err.code) == (500, 'map_dir_invalid')


def test_names_of_a_non_string_directory_is_empty():
    saver, _, _ = _saver(None)
    assert saver.names() == []


# --- names that try to escape (direct use of the saver) ----------------------------------

@pytest.mark.parametrize('name', ['../x', 'a/b', '.', '..', '', ' ', '/etc/passwd', 'a b',
                                  'x' * 41, 'a\x00b', 'a\n', None, 5, ['a']])
def test_valid_map_name_and_the_saver_reject_traversal_names(map_dir, name):
    with pytest.raises(ValueError):
        valid_map_name(name)
    saver, port, _ = _saver(map_dir)
    # Defence in depth: even if a caller forgot to validate, nothing is written or sent.
    with pytest.raises((ValueError, ApiError)):
        saver.save(name, True)
    assert port.saves() == []
    assert _files(map_dir) == []
    assert _files(os.path.dirname(map_dir)) == ['maps']


@pytest.mark.parametrize('name', ['-h', '--help', '-rf', '-', '-x'])
def test_option_like_names_are_rejected_by_the_saver_and_nothing_reaches_slam_toolbox(
        map_dir, name):
    saver, port, _ = _saver(map_dir)
    with pytest.raises(ValueError):
        valid_map_name(name)
    with pytest.raises((ValueError, ApiError)):
        saver.save(name, False)
    assert port.saves() == [] and _files(map_dir) == []


# --- the module does not log -------------------------------------------------------------

def test_web_server_does_not_use_logging_events_go_through_on_event():
    with open(web_server.__file__, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split('.')[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split('.')[0])
    assert 'logging' not in imported
    calls = {n.func.id for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}
    assert 'print' not in calls


def test_the_saver_works_without_an_event_listener(map_dir):
    saver = MapSaver(map_dir, FakePort())
    assert saver.save('lab', False)['ok'] is True


# --- HTTP: POST /api/map/save and GET /api/maps ------------------------------------------

def _client(map_dir, port='default', mode='mapping', nav=None, events=None, clock=None,
            min_free_mb=0, min_interval_s=0.0, **kwargs):
    """Build a TestClient; by default no disk quota, no rate limit and the real clock."""
    port = FakePort() if port == 'default' else port
    store = StateStore()
    if mode is not None:
        store.update(mode=mode)

    def saver_with_clock(*args, **saver_kwargs):
        return MapSaver(*args, clock=clock, **saver_kwargs)

    with mock.patch.object(web_server, 'MapSaver',
                           saver_with_clock if clock is not None else MapSaver):
        app = create_app(store, TOKEN, nav_port=nav, map_port=port, map_dir=map_dir,
                         on_event=(lambda level, text: events.append((level, text)))
                         if events is not None else None,
                         map_min_free_mb=min_free_mb, map_min_interval_s=min_interval_s)
    return TestClient(app, **kwargs), port


def _post(client, body, headers=AUTH, raw=None):
    if raw is not None:
        return client.post('/api/map/save', content=raw,
                           headers={**headers, 'Content-Type': 'application/json'})
    return client.post('/api/map/save', json=body, headers=headers)


def test_ca_http_save_success(map_dir):
    client, port = _client(map_dir)
    response = _post(client, {'name': 'lab'})
    assert response.status_code == 200
    assert response.json() == {'ok': True, 'name': 'lab', 'posegraph': True}
    assert os.path.isfile(os.path.join(map_dir, 'lab.yaml'))


def test_http_save_with_overwrite_true_replaces_a_map(map_dir):
    _old_map(map_dir)
    client, _ = _client(map_dir)
    assert _post(client, {'name': 'lab'}).json()['error'] == 'map_exists'
    response = _post(client, {'name': 'lab', 'overwrite': True})
    assert response.status_code == 200
    assert _read(os.path.join(map_dir, 'lab.yaml')) == b'new-yaml'


def test_ca_http_map_exists_is_409_with_only_a_stray_data_file(map_dir):
    _write(os.path.join(map_dir, 'lab.data'))
    client, port = _client(map_dir)
    response = _post(client, {'name': 'lab', 'overwrite': False})
    assert response.status_code == 409
    assert response.json()['error'] == 'map_exists'
    assert port.saves() == []


@pytest.mark.parametrize('mode', ['map', None, '', 'nav'])
def test_ca_http_wrong_mode_answers_409_and_never_reaches_slam_toolbox(map_dir, mode):
    client, port = _client(map_dir, mode=mode)
    response = _post(client, {'name': 'lab'})
    assert response.status_code == 409
    assert response.json()['error'] == 'wrong_mode'
    assert [c for c in port.calls if c[0] != 'ready'] == []
    assert _files(map_dir) == []


def test_http_mapping_mode_is_accepted(map_dir):
    client, _ = _client(map_dir, mode='mapping')
    assert _post(client, {'name': 'lab'}).status_code == 200


def test_ca_http_slam_unavailable_is_503(map_dir):
    port = FakePort()
    port.is_ready = False
    client, _ = _client(map_dir, port)
    response = _post(client, {'name': 'lab'})
    assert response.status_code == 503
    assert response.json()['error'] == 'slam_unavailable'


def test_http_without_a_map_port_is_503_slam_unavailable(map_dir):
    client, _ = _client(map_dir, port=None)
    assert _post(client, {'name': 'lab'}).json()['error'] == 'slam_unavailable'
    assert client.get('/api/maps', headers=AUTH).status_code == 503


def test_http_without_a_map_dir_is_503_slam_unavailable():
    app = create_app(StateStore(), TOKEN, map_port=FakePort(), map_dir=None)
    client = TestClient(app)
    assert client.get('/api/maps', headers=AUTH).status_code == 503


def test_ca_http_timeout_is_504_and_the_next_save_is_409(map_dir):
    port = _timeout_port(partial=False)
    client, _ = _client(map_dir, port)
    first = _post(client, {'name': 'lab'})
    assert (first.status_code, first.json()['error']) == (504, 'save_timeout')
    second = _post(client, {'name': 'other'})
    assert (second.status_code, second.json()['error']) == (409, 'save_in_progress')


def test_ca_http_save_failed_is_500(map_dir):
    port = FakePort()
    port.save_hook = lambda base: True
    client, _ = _client(map_dir, port)
    response = _post(client, {'name': 'lab'})
    assert (response.status_code, response.json()['error']) == (500, 'save_failed')


def test_ca_http_map_dir_invalid_is_500_and_does_not_leak_the_path(tmp_path):
    bad = os.path.join(os.path.realpath(str(tmp_path)), 'bad dir')
    client, port = _client(bad)
    response = _post(client, {'name': 'lab'})
    assert (response.status_code, response.json()['error']) == (500, 'map_dir_invalid')
    assert 'bad dir' not in response.text
    assert port.saves() == []


def test_ca_http_a_raising_port_gives_a_json_500_without_a_traceback(map_dir):
    port = FakePort()

    def boom(base):
        raise RuntimeError(SECRET_TEXT)

    port.save_hook = boom
    client, _ = _client(map_dir, port)
    response = _post(client, {'name': 'lab'})
    assert (response.status_code, response.json()['error']) == (500, 'save_failed')
    assert SECRET_TEXT not in response.text
    assert 'Traceback' not in response.text
    assert 'RuntimeError' not in response.text


def test_http_a_raising_ready_gives_503_slam_unavailable_json_and_does_not_wedge_the_saver(
        map_dir):
    port = FakePort()
    state = {'boom': True}
    original = port.ready

    def ready():
        if state['boom']:
            raise RuntimeError(SECRET_TEXT)
        return original()

    port.ready = ready
    client, _ = _client(map_dir, port, raise_server_exceptions=False)
    response = _post(client, {'name': 'lab'})
    assert response.status_code == 503
    assert response.headers['content-type'].startswith('application/json')
    assert response.json()['error'] == 'slam_unavailable'
    assert SECRET_TEXT not in response.text and 'Traceback' not in response.text
    assert 'RuntimeError' not in response.text
    assert port.saves() == [] and _files(map_dir) == []
    state['boom'] = False
    assert _post(client, {'name': 'lab'}).status_code == 200   # lock was released


# body validation

@pytest.mark.parametrize('body', [
    {'name': 'lab', 'extra': 1}, {'name': 'lab', 'overwrite': False, 'x': None},
    {'overwrite': True}, {}, [], ['name'], 'lab', 5, None, True])
def test_ca_http_invalid_body_is_400_invalid_body(map_dir, body):
    client, port = _client(map_dir)
    response = _post(client, body)
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_body'
    assert port.saves() == []


@pytest.mark.parametrize('name', [True, False, 0, 5, None, ['lab'], {'a': 1}, 1.5, 10 ** 400,
                                  '', 'x' * 41, '../x', 'a/b', '.', '..', 'a b', 'ñ', '/abs'])
def test_ca_http_invalid_name_is_400_invalid_name(map_dir, name):
    client, port = _client(map_dir)
    response = _post(client, raw=json.dumps({'name': name}), body=None)
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_name'
    assert port.saves() == []
    assert _files(os.path.dirname(map_dir)) == ['maps']


@pytest.mark.parametrize('overwrite', [1, 0, 'true', 'yes', None, [], {}, 1.0])
def test_ca_http_overwrite_must_be_a_real_boolean(map_dir, overwrite):
    _old_map(map_dir)
    client, port = _client(map_dir)
    response = _post(client, {'name': 'lab', 'overwrite': overwrite})
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_body'
    assert port.saves() == []
    assert _read(os.path.join(map_dir, 'lab.yaml')) == b'old.yaml'


@pytest.mark.parametrize('name', ['-h', '--help', '-', '-x', '-rf'])
def test_http_option_like_names_are_400_invalid_name_and_never_reach_slam_toolbox(
        map_dir, name):
    client, port = _client(map_dir)
    response = _post(client, {'name': name})
    assert (response.status_code, response.json()['error']) == (400, 'invalid_name')
    assert port.saves() == [] and _files(map_dir) == []


@pytest.mark.parametrize('body', ['{"name": "lab"', "{'name': 'lab'}", 'garbage', '{"name": NaN}'])
def test_ca_http_broken_json_is_400_invalid_json(map_dir, body):
    client, port = _client(map_dir)
    response = _post(client, None, raw=body)
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_json'
    assert port.saves() == []


def test_ca_http_body_over_1_kb_is_400_body_too_large(map_dir):
    client, port = _client(map_dir)
    response = _post(client, {'name': 'lab', 'overwrite': False, 'pad': 'x' * 2000})
    assert response.status_code == 400
    assert response.json()['error'] == 'body_too_large'
    assert port.saves() == []


def test_http_body_over_1_kb_without_content_length_is_400_body_too_large(map_dir):
    client, port = _client(map_dir)
    chunks = iter([b'{"name": "', b'x' * 600, b'", "pad": "', b'y' * 600, b'"}'])
    response = client.post('/api/map/save', content=chunks,
                           headers={**AUTH, 'Content-Type': 'application/json'})
    assert response.status_code == 400
    assert response.json()['error'] == 'body_too_large'
    assert port.saves() == []


def test_http_empty_body_is_400_invalid_body(map_dir):
    client, port = _client(map_dir)
    response = client.post('/api/map/save', headers=AUTH)
    assert (response.status_code, response.json()['error']) == (400, 'invalid_body')


# authentication comes before everything else

@pytest.mark.parametrize('headers', [{}, {'Authorization': 'Bearer wrong'},
                                     {'Authorization': f'Basic {TOKEN}'},
                                     {'Authorization': 'Bearer '}])
@pytest.mark.parametrize('body', [{'name': 'lab'}, {'bad': 1}, 'x' * 5000, None])
def test_ca_http_token_is_checked_before_the_body_and_the_mode(map_dir, headers, body):
    client, port = _client(map_dir, mode='map')   # wrong mode too: still 401, not 409
    response = (_post(client, None, headers, raw=body) if isinstance(body, str)
                else _post(client, body, headers))
    assert response.status_code == 401
    assert response.json()['error'] == 'unauthorized'
    assert port.calls == []
    assert _files(map_dir) == []


def test_http_get_maps_requires_the_token(map_dir):
    client, _ = _client(map_dir)
    assert client.get('/api/maps').status_code == 401
    assert client.get('/api/maps', headers={'Authorization': 'Bearer nope'}).status_code == 401


# GET /api/maps

def test_ca_http_get_maps_returns_sorted_names(map_dir):
    client, _ = _client(map_dir)
    _post(client, {'name': 'b'})
    _post(client, {'name': 'a'})
    response = client.get('/api/maps', headers=AUTH)
    assert response.status_code == 200
    assert response.json() == {'maps': ['a', 'b']}


def test_http_get_maps_works_in_any_mode(map_dir):
    _old_map(map_dir)
    client, _ = _client(map_dir, mode='map')
    assert client.get('/api/maps', headers=AUTH).json() == {'maps': ['lab']}


def test_http_get_maps_at_most_200(map_dir):
    for i in range(210):
        _write(os.path.join(map_dir, f'm{i:03d}.yaml'))
        _write(os.path.join(map_dir, f'm{i:03d}.pgm'))
    client, _ = _client(map_dir)
    maps = client.get('/api/maps', headers=AUTH).json()['maps']
    assert maps == [f'm{i:03d}' for i in range(200)]


def test_http_get_maps_missing_directory_is_empty(tmp_path):
    client, _ = _client(os.path.join(os.path.realpath(str(tmp_path)), 'missing'))
    assert client.get('/api/maps', headers=AUTH).json() == {'maps': []}


def test_http_get_maps_invalid_directory_is_500_map_dir_invalid(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    os.mkdir('rel')
    client, _ = _client('rel')
    response = client.get('/api/maps', headers=AUTH)
    assert (response.status_code, response.json()['error']) == (500, 'map_dir_invalid')


# --- /api/stop is never delayed by a long save -------------------------------------------

class StopNav(NavPort):
    def __init__(self):
        self.stops = 0
        self.stopped = threading.Event()

    def stop(self):
        self.stops += 1
        self.stopped.set()


def test_ca_stop_is_not_delayed_while_a_save_is_blocked_inside_slam_toolbox(map_dir):
    entered, release = threading.Event(), threading.Event()
    port = FakePort()

    def blocking(base):
        entered.set()
        assert release.wait(WAIT_S)
        _write(base + '.yaml')
        _write(base + '.pgm')
        return True

    port.save_hook = blocking
    nav = StopNav()
    client, _ = _client(map_dir, port, nav=nav)
    saved = {}

    def save():
        saved['response'] = _post(client, {'name': 'lab'})

    stopped = {}

    def stop():
        stopped['response'] = client.post('/api/stop', headers=AUTH)

    saver_thread = threading.Thread(target=save)
    stop_thread = threading.Thread(target=stop)
    saver_thread.start()
    try:
        assert entered.wait(WAIT_S)
        stop_thread.start()
        stop_thread.join(WAIT_S)
        # The save is still blocked, yet the stop already answered and reached the NavPort.
        assert not stop_thread.is_alive(), 'POST /api/stop waited for the map save'
        assert stopped['response'].status_code == 200
        assert nav.stops == 1 and nav.stopped.is_set()
        assert 'response' not in saved
    finally:
        release.set()
        saver_thread.join(WAIT_S)
        stop_thread.join(WAIT_S)
    assert saved['response'].status_code == 200


def test_http_concurrent_save_gets_409_while_stop_still_works(map_dir):
    entered, release = threading.Event(), threading.Event()
    port = FakePort()

    def blocking(base):
        entered.set()
        assert release.wait(WAIT_S)
        _write(base + '.yaml')
        _write(base + '.pgm')
        return True

    port.save_hook = blocking
    nav = StopNav()
    client, _ = _client(map_dir, port, nav=nav)
    worker = threading.Thread(target=lambda: _post(client, {'name': 'first'}))
    worker.start()
    try:
        assert entered.wait(WAIT_S)
        second = _post(client, {'name': 'second'})
        assert (second.status_code, second.json()['error']) == (409, 'save_in_progress')
        assert client.post('/api/stop', headers=AUTH).status_code == 200
    finally:
        release.set()
        worker.join(WAIT_S)
    assert len(port.saves()) == 1


# --- events -------------------------------------------------------------------------------

def test_http_events_reach_on_event_with_a_valid_level(map_dir):
    events = []
    client, _ = _client(map_dir, events=events)
    _post(client, {'name': 'lab'})
    assert events and all(level in ('info', 'warning', 'error') for level, _ in events)
    assert any('lab' in text for _, text in events)


# =========================================================================================
# Phase 5 security fixes: name rules, .bak protection, quotas, rate limit, ready(), paths.
# Behaviour fixed here is the SAFE one; nothing sleeps (monotonic clock injected).
# =========================================================================================

ALL_EXTS = ('.yaml', '.pgm', '.png', '.posegraph', '.data')
QUOTA_MB = 200
QUOTA_BYTES = QUOTA_MB * 1024 * 1024
DiskUsage = collections.namedtuple('DiskUsage', 'total used free')


def _snapshot(directory):
    """Return {file name: bytes} of every file of `directory` (to compare byte by byte)."""
    return {name: _read(os.path.join(directory, name)) for name in _files(directory)}


def _slam_calls(port):
    """Return the calls that really ask slam_toolbox to write something."""
    return [c for c in port.calls if c[0] in ('save', 'serialize')]


def _fill_with_maps(directory, count, prefix='m'):
    for i in range(count):
        _write(os.path.join(directory, f'{prefix}{i:03d}.yaml'), b'y')
        _write(os.path.join(directory, f'{prefix}{i:03d}.pgm'), b'p')


def _fake_disk(monkeypatch, free_bytes):
    calls = []

    def disk_usage(path):
        calls.append(path)
        return DiskUsage(10 * free_bytes + 1, 1, free_bytes)

    monkeypatch.setattr(web_server.shutil, 'disk_usage', disk_usage)
    return calls


# --- names: reserved, dash, length (CA: names) -------------------------------------------

@pytest.mark.parametrize('name', ['map', 'Map', 'MAP', 'mAp'])
def test_http_reserved_name_map_in_any_case_is_400_reserved_name(map_dir, name):
    client, port = _client(map_dir)
    response = _post(client, {'name': name})
    assert (response.status_code, response.json()['error']) == (400, 'reserved_name')
    assert port.calls == [] and _files(map_dir) == []


@pytest.mark.parametrize('name', ['map', 'Map', 'MAP'])
def test_reserved_name_is_refused_even_with_overwrite_and_existing_files(map_dir, name):
    client, port = _client(map_dir)
    _write(os.path.join(map_dir, name + '.yaml'), b'precious')
    response = _post(client, {'name': name, 'overwrite': True})
    assert (response.status_code, response.json()['error']) == (400, 'reserved_name')
    assert _snapshot(map_dir) == {name + '.yaml': b'precious'} and port.calls == []


@pytest.mark.parametrize('name', ['maps', 'map1', 'my_map', 'mapa', '_map'])
def test_names_that_only_contain_map_are_not_reserved(map_dir, name):
    client, _ = _client(map_dir)
    assert _post(client, {'name': name}).status_code == 200


@pytest.mark.parametrize('name', ['-x', '-', '--help', '', 'x' * 41, 'a/b', '.', '..', 'ñ'])
def test_http_invalid_names_are_400_invalid_name_and_never_reach_slam_toolbox(map_dir, name):
    client, port = _client(map_dir)
    response = _post(client, {'name': name})
    assert (response.status_code, response.json()['error']) == (400, 'invalid_name')
    assert port.calls == [] and _files(map_dir) == []
    assert _files(os.path.dirname(map_dir)) == ['maps']


def test_http_a_40_character_name_is_accepted_and_the_final_path_is_absolute_and_confined(
        map_dir):
    name = 'a' + 'b_-' * 13       # 40 characters, contains '-' and '_' inside
    assert len(name) == 40
    client, port = _client(map_dir)
    response = _post(client, {'name': name})
    assert response.status_code == 200
    [(_, base, _)] = port.saves()
    assert base == os.path.join(map_dir, name)
    assert base.startswith('/') and os.path.dirname(base) == map_dir
    assert MAP_PATH_PATTERN.fullmatch(base) is not None


def test_http_get_maps_never_lists_reserved_or_dash_names_even_if_files_exist(map_dir):
    for name in ('map', 'Map', 'MAP', '-x', '--help'):
        _write(os.path.join(map_dir, name + '.yaml'))
        _write(os.path.join(map_dir, name + '.pgm'))
    _old_map(map_dir, 'lab')
    client, _ = _client(map_dir)
    response = client.get('/api/maps', headers=AUTH)
    assert response.json() == {'maps': ['lab']}


# --- .bak protection (CA: backup_pending) ------------------------------------------------

@pytest.mark.parametrize('bak_ext', ALL_EXTS)
def test_overwrite_with_a_preexisting_bak_of_any_extension_is_409_and_touches_nothing(
        map_dir, bak_ext):
    _old_map(map_dir)
    _write(os.path.join(map_dir, 'lab' + bak_ext + '.bak'), b'IRREPLACEABLE-' + bak_ext.encode())
    before = _snapshot(map_dir)
    saver, port, events = _saver(map_dir)
    err = _error(saver.save, 'lab', True)
    assert (err.status_code, err.code) == (409, 'backup_pending')
    assert _snapshot(map_dir) == before            # byte for byte, no rename, no new file
    assert port.calls == [('ready',)]              # slam_toolbox was never asked to write
    assert any(level == 'error' for level, _ in events)


@pytest.mark.parametrize('bak_ext', ALL_EXTS)
def test_http_overwrite_with_a_preexisting_bak_is_409_backup_pending(map_dir, bak_ext):
    _old_map(map_dir)
    _write(os.path.join(map_dir, 'lab' + bak_ext + '.bak'), b'IRREPLACEABLE')
    before = _snapshot(map_dir)
    client, port = _client(map_dir)
    response = _post(client, {'name': 'lab', 'overwrite': True})
    assert (response.status_code, response.json()['error']) == (409, 'backup_pending')
    assert _snapshot(map_dir) == before and _slam_calls(port) == []


def test_a_bak_of_another_map_does_not_block_the_overwrite(map_dir):
    _old_map(map_dir)
    _write(os.path.join(map_dir, 'other.yaml.bak'), b'unrelated')
    saver, _, _ = _saver(map_dir)
    assert saver.save('lab', True)['ok'] is True
    assert _read(os.path.join(map_dir, 'other.yaml.bak')) == b'unrelated'


@pytest.mark.parametrize('overwrite', [False, True])
def test_a_new_save_with_an_orphan_bak_and_no_map_is_not_blocked_and_keeps_the_bak(
        map_dir, overwrite):
    # Current behaviour: the .bak check applies only when there is a map to overwrite.
    _write(os.path.join(map_dir, 'lab.yaml.bak'), b'orphan')
    saver, _, _ = _saver(map_dir)
    assert saver.save('lab', overwrite)['ok'] is True
    assert _read(os.path.join(map_dir, 'lab.yaml.bak')) == b'orphan'
    assert _read(os.path.join(map_dir, 'lab.yaml')) == b'new-yaml'


def test_timeout_then_late_write_then_second_overwrite_never_loses_the_original_backup(
        map_dir):
    # Exact sequence of the finding: overwrite -> slam_toolbox times out (files moved to
    # .bak, nothing restored) -> slam_toolbox finishes writing late -> pause ends -> a new
    # overwrite must NOT replace the original .bak with the late (possibly partial) files.
    _old_map(map_dir)
    clock = Clock(1000.0)
    port = FakePort()

    def timing_out(base):
        raise SlamTimeoutError('late')

    port.save_hook = timing_out
    saver, _, _ = _saver(map_dir, port, clock)
    assert _error(saver.save, 'lab', True).code == 'save_timeout'
    original_baks = {n: _read(os.path.join(map_dir, n)) for n in _files(map_dir)
                     if n.endswith('.bak')}
    assert original_baks == {'lab.yaml.bak': b'old.yaml', 'lab.pgm.bak': b'old.pgm',
                             'lab.posegraph.bak': b'old.posegraph',
                             'lab.data.bak': b'old.data'}
    _write(os.path.join(map_dir, 'lab.yaml'), b'LATE-PARTIAL-yaml')   # the late writer
    _write(os.path.join(map_dir, 'lab.pgm'), b'LATE-PARTIAL-pgm')
    clock.now += MAP_SAVE_COOLDOWN_S + 1
    port.save_hook = None
    saves_before = len(port.saves())
    err = _error(saver.save, 'lab', True)
    assert (err.status_code, err.code) == (409, 'backup_pending')
    assert len(port.saves()) == saves_before
    assert {n: _read(os.path.join(map_dir, n)) for n in _files(map_dir)
            if n.endswith('.bak')} == original_baks
    assert _read(os.path.join(map_dir, 'lab.yaml')) == b'LATE-PARTIAL-yaml'


# --- quota: number of maps (CA: map_limit) -----------------------------------------------

def test_200_saved_maps_plus_a_new_one_is_409_map_limit_and_slam_toolbox_is_not_asked(
        map_dir):
    _fill_with_maps(map_dir, 200)
    before = _snapshot(map_dir)
    saver, port, events = _saver(map_dir)
    err = _error(saver.save, 'brand_new', False)
    assert (err.status_code, err.code) == (409, 'map_limit')
    assert _snapshot(map_dir) == before
    assert _slam_calls(port) == []
    assert any(level == 'warning' for level, _ in events)


def test_199_saved_maps_still_allow_a_new_one(map_dir):
    _fill_with_maps(map_dir, 199)
    saver, _, _ = _saver(map_dir)
    assert saver.save('brand_new', False)['ok'] is True


def test_overwriting_an_existing_map_passes_with_200_maps_stored(map_dir):
    _fill_with_maps(map_dir, 200)
    saver, port, _ = _saver(map_dir)
    assert saver.save('m000', True)['ok'] is True
    assert len(port.saves()) == 1
    assert _read(os.path.join(map_dir, 'm000.yaml')) == b'new-yaml'


def test_http_map_limit_is_409_and_does_not_leak_the_count_of_files(map_dir):
    _fill_with_maps(map_dir, 200)
    client, port = _client(map_dir)
    response = _post(client, {'name': 'brand_new'})
    assert (response.status_code, response.json()['error']) == (409, 'map_limit')
    assert _slam_calls(port) == []


def test_the_map_limit_does_not_count_reserved_or_invalid_names_as_maps(map_dir):
    # list_maps ignores them, so planting `map.*` / `-x.*` files cannot fill the quota.
    _fill_with_maps(map_dir, 199)
    for name in ('map', 'Map', '-x'):
        _write(os.path.join(map_dir, name + '.yaml'))
        _write(os.path.join(map_dir, name + '.pgm'))
    saver, _, _ = _saver(map_dir)
    assert saver.save('brand_new', False)['ok'] is True


# --- quota: free disk (CA: insufficient_storage) -----------------------------------------

def test_free_space_below_the_minimum_is_507_insufficient_storage_and_nothing_is_written(
        map_dir, monkeypatch):
    _old_map(map_dir)
    before = _snapshot(map_dir)
    _fake_disk(monkeypatch, QUOTA_BYTES - 1)
    saver, port, events = _saver(map_dir, min_free_mb=QUOTA_MB)
    err = _error(saver.save, 'lab', True)
    assert (err.status_code, err.code) == (507, 'insufficient_storage')
    assert _snapshot(map_dir) == before              # not even moved to .bak
    assert _slam_calls(port) == []
    assert any(level == 'error' for level, _ in events)


def test_free_space_exactly_at_the_minimum_passes(map_dir, monkeypatch):
    calls = _fake_disk(monkeypatch, QUOTA_BYTES)
    saver, _, _ = _saver(map_dir, min_free_mb=QUOTA_MB)
    assert saver.save('lab', False)['ok'] is True
    assert calls and all(os.path.isabs(c) for c in calls)


def test_the_minimum_is_in_megabytes_of_1024_squared(map_dir, monkeypatch):
    _fake_disk(monkeypatch, 5 * 1000 * 1000)       # 5 MB decimal is below 5 MiB
    saver, _, _ = _saver(map_dir, min_free_mb=5)
    assert _error(saver.save, 'lab', False).code == 'insufficient_storage'


def test_http_insufficient_storage_is_507(map_dir, monkeypatch):
    _fake_disk(monkeypatch, 0)
    client, port = _client(map_dir, min_free_mb=QUOTA_MB)
    response = _post(client, {'name': 'lab'})
    assert (response.status_code, response.json()['error']) == (507, 'insufficient_storage')
    assert _slam_calls(port) == []


def test_http_default_quotas_are_200_mb_and_5_seconds():
    assert (web_server.MAP_MIN_FREE_MB_DEFAULT, web_server.MAP_MIN_INTERVAL_S_DEFAULT) == (
        200, 5.0)
    assert web_server.MAP_MAX_SAVED == 200


def test_a_failing_disk_usage_check_answers_500_map_dir_invalid_and_saves_nothing(
        map_dir, monkeypatch):
    def broken(path):
        raise OSError(SECRET_TEXT)

    monkeypatch.setattr(web_server.shutil, 'disk_usage', broken)
    saver, port, _ = _saver(map_dir, min_free_mb=QUOTA_MB)
    err = _error(saver.save, 'lab', False)
    assert (err.status_code, err.code) == (500, 'map_dir_invalid')
    assert SECRET_TEXT not in err.detail
    assert _slam_calls(port) == []


def test_the_map_limit_is_checked_before_the_free_space_and_overwrites_skip_the_limit(
        map_dir, monkeypatch):
    # Both quotas are checked before slam_toolbox is asked: map_limit first, then storage.
    _fill_with_maps(map_dir, 200)
    _fake_disk(monkeypatch, 0)
    saver, port, _ = _saver(map_dir, min_free_mb=QUOTA_MB)
    assert _error(saver.save, 'brand_new', False).code == 'map_limit'
    assert _error(saver.save, 'm001', True).code == 'insufficient_storage'
    assert _slam_calls(port) == []


# --- rate limit (CA: save_rate_limited) --------------------------------------------------

def test_rate_limit_429_inside_the_interval_and_ok_after_it(map_dir):
    clock = Clock(1000.0)
    saver, port, _ = _saver(map_dir, clock=clock, min_interval_s=5.0)
    assert saver.save('a', False)['ok'] is True
    for elapsed in (0.0, 1.0, 4.99):
        clock.now = 1000.0 + elapsed
        err = _error(saver.save, 'b', False)
        assert (err.status_code, err.code) == (429, 'save_rate_limited')
    assert len(port.saves()) == 1 and not os.path.exists(os.path.join(map_dir, 'b.yaml'))
    clock.now = 1000.0 + 5.0
    assert saver.save('b', False)['ok'] is True
    assert len(port.saves()) == 2


def test_the_interval_restarts_after_every_accepted_save(map_dir):
    clock = Clock(0.0)
    saver, _, _ = _saver(map_dir, clock=clock, min_interval_s=5.0)
    saver.save('a', False)
    clock.now = 5.0
    saver.save('b', False)
    clock.now = 9.0
    assert _error(saver.save, 'c', False).code == 'save_rate_limited'
    clock.now = 10.0
    assert saver.save('c', False)['ok'] is True


def test_a_zero_interval_allows_back_to_back_saves(map_dir):
    saver, _, _ = _saver(map_dir, min_interval_s=0.0)
    assert saver.save('a', False)['ok'] and saver.save('b', False)['ok']


def test_rate_limit_is_the_same_for_overwrite_and_new_maps(map_dir):
    clock = Clock(10.0)
    saver, _, _ = _saver(map_dir, clock=clock, min_interval_s=5.0)
    saver.save('a', False)
    assert _error(saver.save, 'a', True).code == 'save_rate_limited'


def test_the_rate_limit_also_counts_a_save_that_fails_afterwards(map_dir):
    # Current behaviour, documented: the interval starts when slam_toolbox is asked, so a
    # failed save (500) is followed by 429 for the next `min_interval_s`.
    clock = Clock(100.0)
    port = FakePort()
    port.save_result = False
    saver, _, _ = _saver(map_dir, port, clock, min_interval_s=5.0)
    assert _error(saver.save, 'a', False).code == 'save_failed'
    port.save_result = True
    clock.now = 102.0
    assert _error(saver.save, 'a', False).code == 'save_rate_limited'
    clock.now = 105.0
    assert saver.save('a', False)['ok'] is True


def test_refusals_before_slam_toolbox_is_asked_do_not_start_the_interval(map_dir):
    # map_exists, backup_pending, map_limit and insufficient storage consume no interval.
    _old_map(map_dir)
    clock = Clock(100.0)
    saver, _, _ = _saver(map_dir, clock=clock, min_interval_s=5.0)
    assert _error(saver.save, 'lab', False).code == 'map_exists'
    assert saver.save('fresh', False)['ok'] is True


def test_the_30_second_pause_after_a_timeout_wins_over_the_rate_limit(map_dir):
    clock = Clock(1000.0)
    port = _timeout_port(partial=False)
    saver, _, _ = _saver(map_dir, port, clock, min_interval_s=5.0)
    assert _error(saver.save, 'a', False).code == 'save_timeout'
    port.save_hook = None
    for elapsed in (0.0, 4.0, 6.0, 20.0, MAP_SAVE_COOLDOWN_S - 0.5):
        clock.now = 1000.0 + elapsed
        err = _error(saver.save, 'b', False)
        assert (err.status_code, err.code) == (409, 'save_in_progress')
    assert len(port.saves()) == 1
    clock.now = 1000.0 + MAP_SAVE_COOLDOWN_S + 0.5
    assert saver.save('b', False)['ok'] is True


def test_http_rate_limit_429_with_an_injected_clock(map_dir):
    clock = Clock(500.0)
    client, port = _client(map_dir, clock=clock, min_interval_s=5.0)
    assert _post(client, {'name': 'a'}).status_code == 200
    clock.now = 503.0
    response = _post(client, {'name': 'b'})
    assert (response.status_code, response.json()['error']) == (429, 'save_rate_limited')
    assert len(port.saves()) == 1
    clock.now = 505.0
    assert _post(client, {'name': 'b'}).status_code == 200


# --- order of the checks (token -> body -> mode -> quotas -> slam_toolbox) ---------------

def test_http_token_is_checked_before_the_quotas(map_dir, monkeypatch):
    _fill_with_maps(map_dir, 200)
    _fake_disk(monkeypatch, 0)
    client, port = _client(map_dir, min_free_mb=QUOTA_MB)
    response = _post(client, {'name': 'brand_new'}, headers={})
    assert (response.status_code, response.json()['error']) == (401, 'unauthorized')
    assert port.calls == []


def test_http_body_is_validated_before_the_quotas(map_dir, monkeypatch):
    _fill_with_maps(map_dir, 200)
    _fake_disk(monkeypatch, 0)
    client, port = _client(map_dir, min_free_mb=QUOTA_MB)
    for body, error in (({'name': 'map'}, 'reserved_name'), ({'name': '-x'}, 'invalid_name'),
                        ({'nombre': 'a'}, 'invalid_body')):
        response = _post(client, body)
        assert (response.status_code, response.json()['error']) == (400, error)
    assert port.calls == []


def test_http_mode_is_checked_before_the_quotas(map_dir, monkeypatch):
    _fill_with_maps(map_dir, 200)
    _fake_disk(monkeypatch, 0)
    client, port = _client(map_dir, mode='map', min_free_mb=QUOTA_MB)
    response = _post(client, {'name': 'brand_new'})
    assert (response.status_code, response.json()['error']) == (409, 'wrong_mode')
    assert port.calls == []


@pytest.mark.parametrize('scenario', ['map_limit', 'insufficient_storage', 'rate_limited',
                                      'backup_pending', 'map_exists'])
def test_no_quota_or_precondition_refusal_asks_slam_toolbox_to_write(
        map_dir, monkeypatch, scenario):
    clock = Clock(0.0)
    name, overwrite = 'brand_new', False
    if scenario == 'map_limit':
        _fill_with_maps(map_dir, 200)
    elif scenario == 'insufficient_storage':
        _fake_disk(monkeypatch, 0)
    elif scenario == 'backup_pending':
        _old_map(map_dir)
        _write(os.path.join(map_dir, 'lab.yaml.bak'))
        name, overwrite = 'lab', True
    elif scenario == 'map_exists':
        _old_map(map_dir)
        name = 'lab'
    client, port = _client(map_dir, clock=clock, min_free_mb=QUOTA_MB, min_interval_s=5.0)
    if scenario == 'rate_limited':
        assert _post(client, {'name': 'first'}).status_code == 200
        port.calls.clear()
    response = _post(client, {'name': name, 'overwrite': overwrite})
    expected = {'map_limit': (409, 'map_limit'), 'insufficient_storage': (507, scenario),
                'rate_limited': (429, 'save_rate_limited'),
                'backup_pending': (409, scenario), 'map_exists': (409, scenario)}[scenario]
    assert (response.status_code, response.json()['error']) == expected
    assert _slam_calls(port) == []


# --- ready() raising (CA: slam_unavailable, lock released) -------------------------------

def test_a_raising_ready_gives_503_slam_unavailable_and_the_lock_is_free_for_the_next_save(
        map_dir):
    saver, port, events = _saver(map_dir)
    state = {'boom': True}
    original = port.ready

    def ready():
        if state['boom']:
            raise RuntimeError(SECRET_TEXT)
        return original()

    port.ready = ready
    err = _error(saver.save, 'lab', False)
    assert (err.status_code, err.code) == (503, 'slam_unavailable')
    assert SECRET_TEXT not in err.detail
    assert all(SECRET_TEXT not in text for _, text in events)
    assert port.saves() == [] and _files(map_dir) == []
    state['boom'] = False
    assert saver.save('lab', False)['ok'] is True          # lock released, nothing wedged


@pytest.mark.parametrize('error', [OSError(SECRET_TEXT), KeyError(SECRET_TEXT),
                                   TimeoutError(SECRET_TEXT)])
def test_any_exception_of_ready_is_a_503_not_a_500(map_dir, error):
    saver, port, _ = _saver(map_dir)

    def ready():
        raise error

    port.ready = ready
    err = _error(saver.save, 'lab', False)
    assert (err.status_code, err.code) == (503, 'slam_unavailable')


# --- map_dir creation and confinement ----------------------------------------------------

def test_the_first_save_creates_the_map_dir_with_mode_0755(tmp_path):
    real = os.path.realpath(str(tmp_path))
    target = os.path.join(real, 'home', 'mtrb_maps')
    if MAP_PATH_PATTERN.fullmatch(target) is None:
        pytest.skip('tmp_path has unusual characters')
    old_umask = os.umask(0o022)
    try:
        saver, _, _ = _saver(target)
        assert not os.path.exists(target)
        assert saver.save('lab', False)['ok'] is True
    finally:
        os.umask(old_umask)
    assert os.path.isdir(target)
    assert (os.stat(target).st_mode & 0o777) == 0o755


def test_listing_a_not_yet_created_map_dir_does_not_create_it(tmp_path):
    target = os.path.join(os.path.realpath(str(tmp_path)), 'later')
    saver, _, _ = _saver(target)
    assert saver.names() == []
    assert not os.path.exists(target)


def test_http_a_symlinked_map_file_pointing_outside_is_rejected_and_never_written(map_dir):
    outside = os.path.join(os.path.dirname(map_dir), 'victim')
    _write(outside, b'do-not-touch')
    os.symlink(outside, os.path.join(map_dir, 'lab.yaml'))
    client, port = _client(map_dir)
    for overwrite in (False, True):
        response = _post(client, {'name': 'lab', 'overwrite': overwrite})
        assert (response.status_code, response.json()['error']) == (500, 'map_dir_invalid')
    assert _read(outside) == b'do-not-touch' and _slam_calls(port) == []


# --- no absolute server path ever reaches the client -------------------------------------

SENTINEL = 'SENTINELdirQ7x'


@pytest.fixture
def sentinel_dir(tmp_path):
    real = os.path.realpath(str(tmp_path))
    path = os.path.join(real, SENTINEL)
    if MAP_PATH_PATTERN.fullmatch(path) is None:
        pytest.skip('tmp_path has unusual characters')
    os.mkdir(path)
    return path


def _assert_no_server_path(response, directory):
    haystack = response.text + ' ' + ' '.join(f'{k}: {v}' for k, v in response.headers.items())
    for forbidden in (SENTINEL, directory, os.path.dirname(directory), '/Users/', '/private/',
                      'Traceback', '.py'):
        assert forbidden not in haystack, forbidden


def test_error_responses_never_contain_absolute_paths_of_the_server(sentinel_dir, monkeypatch):
    clock = Clock(0.0)
    monkeypatch.setattr(web_server.shutil, 'disk_usage',
                        lambda path: DiskUsage(10 ** 12, 1, 10 ** 12))
    _old_map(sentinel_dir, 'lab')
    _old_map(sentinel_dir, 'baked')
    _write(os.path.join(sentinel_dir, 'baked.yaml.bak'), b'x')
    os.symlink(os.path.join(sentinel_dir, 'lab.yaml'), os.path.join(sentinel_dir, 'ln.yaml'))
    client, port = _client(sentinel_dir, clock=clock, min_interval_s=5.0, min_free_mb=1)
    responses = [
        _post(client, {'name': 'lab'}),                                  # map_exists
        _post(client, {'name': 'baked', 'overwrite': True}),             # backup_pending
        _post(client, {'name': 'ln', 'overwrite': True}),                # map_dir_invalid
        _post(client, {'name': '../' + SENTINEL}),                       # invalid_name
        _post(client, {'name': 'map'}),                                  # reserved_name
        _post(client, {'name': 'x'}),                                    # ok (starts interval)
        _post(client, {'name': 'y'}),                                    # 429
        client.get('/api/maps', headers=AUTH),
    ]
    port.is_ready = False
    clock.now = 100.0
    responses.append(_post(client, {'name': 'z'}))                       # 503
    port.is_ready = True
    port.save_hook = lambda base: (_ for _ in ()).throw(RuntimeError(base))
    clock.now = 200.0
    responses.append(_post(client, {'name': 'boom'}))                    # 500 save_failed
    port.save_hook = _timeout_port().save_hook
    clock.now = 300.0
    responses.append(_post(client, {'name': 'slow'}))                    # 504
    assert [r.status_code for r in responses[:9]] == [409, 409, 500, 400, 400, 200, 429, 200,
                                                      503]
    assert [r.status_code for r in responses[9:]] == [500, 504]
    for response in responses:
        _assert_no_server_path(response, sentinel_dir)


def test_a_bad_map_dir_error_does_not_contain_the_configured_path(tmp_path):
    bad = os.path.join(os.path.realpath(str(tmp_path)), SENTINEL + ' with space')
    client, _ = _client(bad)
    for response in (_post(client, {'name': 'lab'}), client.get('/api/maps', headers=AUTH)):
        assert response.status_code in (500, 200)
        _assert_no_server_path(response, bad)


# --- /api/stop and teleop are not affected by a long save --------------------------------

class TeleopNav(StopNav):
    def __init__(self):
        super().__init__()
        self.teleops = []
        self.teleoped = threading.Event()

    def teleop(self, client_id, message):
        self.teleops.append(message)
        self.teleoped.set()

    def teleop_disconnect(self, client_id):
        pass


def test_websocket_teleop_and_stop_are_forwarded_while_a_save_is_blocked(map_dir):
    entered, release = threading.Event(), threading.Event()
    port = FakePort()

    def blocking(base):
        entered.set()
        assert release.wait(WAIT_S)
        _write(base + '.yaml')
        _write(base + '.pgm')
        return True

    port.save_hook = blocking
    nav = TeleopNav()
    client, _ = _client(map_dir, port, nav=nav)
    saved = {}
    worker = threading.Thread(target=lambda: saved.update(r=_post(client, {'name': 'lab'})))
    worker.start()
    try:
        assert entered.wait(WAIT_S)
        with client.websocket_connect(f'/ws?token={TOKEN}') as ws:
            ws.send_text(json.dumps({'type': 'teleop', 'vx': 0.1, 'wz': 0.0, 'seq': 1}))
            ws.send_text(json.dumps({'type': 'stop'}))
            ws.send_text('{"type":"sync-probe"}')       # round trip: both were dispatched
            assert json.loads(ws.receive_text())['code'] == 'unsupported_message'
        assert nav.teleops == [{'type': 'teleop', 'vx': 0.1, 'wz': 0.0, 'seq': 1}]
        assert nav.stops == 1
        assert 'r' not in saved                          # the save is still blocked
        assert client.post('/api/stop', headers=AUTH).status_code == 200
        assert nav.stops == 2
    finally:
        release.set()
        worker.join(WAIT_S)
    assert saved['r'].status_code == 200


# --- discarding a partial pose-graph (failed serialization, NOT a timeout) -----------------

def _partial_posegraph_port(how):
    """Return a FakePort whose serialization fails in the given way after writing files."""
    port = FakePort()

    def writer(base):
        _write(base + '.posegraph', b'partial-pg')
        _write(base + '.data', b'partial-data')
        if how == 'raises':
            raise RuntimeError(SECRET_TEXT)
        if how == 'unavailable':
            raise SlamUnavailableError('down')
        if how == 'done_false':
            return False
        if how == 'empty_data':  # reported ok but .data is empty: posegraph_saved is false
            _write(base + '.data', b'')
            return True
        raise AssertionError(how)

    port.serialize_hook = writer
    return port


SERIALIZE_FAILURES = ['done_false', 'raises', 'unavailable', 'empty_data']


@pytest.mark.parametrize('how', SERIALIZE_FAILURES)
def test_ca_failed_serialization_deletes_the_new_partial_posegraph_files(map_dir, how):
    saver, _, events = _saver(map_dir, _partial_posegraph_port(how))
    assert saver.save('lab', False) == {'ok': True, 'name': 'lab', 'posegraph': False}
    # The map (yaml + image) stays; no half pose-graph is left to be loaded by `posegraph:=`.
    assert _files(map_dir) == ['lab.pgm', 'lab.yaml']
    assert _read(os.path.join(map_dir, 'lab.yaml')) == b'new-yaml'
    assert any(level == 'warning' for level, _ in events)


@pytest.mark.parametrize('how', SERIALIZE_FAILURES)
def test_ca_overwrite_with_a_failed_serialization_keeps_the_new_map_and_no_stale_files(
        map_dir, how):
    _old_map(map_dir)
    saver, _, _ = _saver(map_dir, _partial_posegraph_port(how))
    assert saver.save('lab', True) == {'ok': True, 'name': 'lab', 'posegraph': False}
    assert _files(map_dir) == ['lab.pgm', 'lab.yaml']
    assert _read(os.path.join(map_dir, 'lab.pgm')) == b'new-pgm'


@pytest.mark.parametrize('how', SERIALIZE_FAILURES)
def test_an_orphan_bak_survives_the_discard_of_a_failed_serialization(map_dir, how):
    # A `*.bak` left by an old timeout does not block a NEW save (no map) and must not be
    # removed by the cleanup of the partial pose-graph.
    _write(os.path.join(map_dir, 'lab.posegraph.bak'), b'precious-pg')
    _write(os.path.join(map_dir, 'lab.data.bak'), b'precious-data')
    saver, _, _ = _saver(map_dir, _partial_posegraph_port(how))
    assert saver.save('lab', False)['posegraph'] is False
    assert _files(map_dir) == ['lab.data.bak', 'lab.pgm', 'lab.posegraph.bak', 'lab.yaml']
    assert _read(os.path.join(map_dir, 'lab.posegraph.bak')) == b'precious-pg'
    assert _read(os.path.join(map_dir, 'lab.data.bak')) == b'precious-data'


def test_ca_serialization_timeout_deletes_nothing_and_starts_the_pause(map_dir):
    port = FakePort()

    def writer(base):
        _write(base + '.posegraph', b'partial-pg')
        _write(base + '.data', b'partial-data')
        raise SlamTimeoutError('late')

    port.serialize_hook = writer
    saver, _, _ = _saver(map_dir, port)
    assert saver.save('lab', False) == {'ok': True, 'name': 'lab', 'posegraph': False}
    # slam_toolbox may still be writing: the files are left alone.
    assert _files(map_dir) == ['lab.data', 'lab.pgm', 'lab.posegraph', 'lab.yaml']
    assert _read(os.path.join(map_dir, 'lab.posegraph')) == b'partial-pg'
    assert _error(saver.save, 'other', False).code == 'save_in_progress'


@pytest.mark.parametrize('how', ['done_false', 'raises', 'unavailable'])
def test_a_symlink_named_like_a_posegraph_file_is_neither_deleted_nor_followed(map_dir, how):
    outside = os.path.join(os.path.dirname(map_dir), 'victim')
    _write(outside, b'do-not-touch')
    port = FakePort()

    def writer(base):
        os.symlink(outside, base + '.posegraph')
        os.symlink(outside, base + '.data')
        if how == 'raises':
            raise RuntimeError('boom')
        if how == 'unavailable':
            raise SlamUnavailableError('down')
        return False

    port.serialize_hook = writer
    saver, _, _ = _saver(map_dir, port)
    assert saver.save('lab', False)['posegraph'] is False
    assert _read(outside) == b'do-not-touch'
    assert os.path.islink(os.path.join(map_dir, 'lab.posegraph'))
    assert os.path.islink(os.path.join(map_dir, 'lab.data'))


def test_a_directory_named_like_a_posegraph_file_is_not_deleted(map_dir):
    port = FakePort()

    def writer(base):
        os.mkdir(base + '.posegraph')
        _write(os.path.join(base + '.posegraph', 'inner'), b'keep')
        os.mkdir(base + '.data')
        return False

    port.serialize_hook = writer
    saver, _, events = _saver(map_dir, port)
    assert saver.save('lab', False)['posegraph'] is False
    assert [text for level, text in events if level == 'error'] == []  # not even attempted
    assert os.path.isdir(os.path.join(map_dir, 'lab.posegraph'))
    assert _read(os.path.join(map_dir, 'lab.posegraph', 'inner')) == b'keep'
    assert os.path.isdir(os.path.join(map_dir, 'lab.data'))


def test_discard_removes_only_regular_posegraph_files_and_never_the_bak_files(map_dir):
    saver, _, _ = _saver(map_dir)
    base = os.path.join(map_dir, 'lab')
    for ext in ('.posegraph', '.data', '.yaml', '.pgm', '.posegraph.bak', '.data.bak',
                '.yaml.bak', '.pgm.bak'):
        _write(base + ext, b'x' + ext.encode())
    saver._discard_partial_posegraph(base)
    assert _files(map_dir) == ['lab.data.bak', 'lab.pgm', 'lab.pgm.bak', 'lab.posegraph.bak',
                               'lab.yaml', 'lab.yaml.bak']
    assert _read(os.path.join(map_dir, 'lab.posegraph.bak')) == b'x.posegraph.bak'


def test_discard_skips_a_file_that_is_not_inside_the_map_directory(map_dir):
    other = os.path.join(os.path.dirname(map_dir), 'elsewhere')
    os.mkdir(other)
    _write(os.path.join(other, 'lab.posegraph'), b'keep')
    saver, _, _ = _saver(map_dir)
    saver._discard_partial_posegraph(os.path.join(other, 'lab'))
    assert _read(os.path.join(other, 'lab.posegraph')) == b'keep'


def test_discard_of_missing_files_is_silent(map_dir):
    saver, _, events = _saver(map_dir)
    saver._discard_partial_posegraph(os.path.join(map_dir, 'ghost'))
    assert events == []


@pytest.mark.parametrize('how', ['done_false', 'raises', 'unavailable'])
def test_a_symlink_to_a_file_inside_the_map_dir_is_not_deleted_either(map_dir, how):
    # The target lives in the same directory, so only the "is a symlink" rule protects it.
    target = os.path.join(map_dir, 'lab-target.txt')
    _write(target, b'do-not-touch')
    port = FakePort()

    def writer(base):
        os.symlink(target, base + '.posegraph')
        os.symlink(target, base + '.data')
        if how == 'raises':
            raise RuntimeError('boom')
        if how == 'unavailable':
            raise SlamUnavailableError('down')
        return False

    port.serialize_hook = writer
    saver, _, _ = _saver(map_dir, port)
    assert saver.save('lab', False)['posegraph'] is False
    assert _read(target) == b'do-not-touch'
    assert os.path.islink(os.path.join(map_dir, 'lab.posegraph'))
    assert os.path.islink(os.path.join(map_dir, 'lab.data'))
