"""Unit tests for mtrb_bringup.web_server (no ROS needed; needs fastapi and httpx)."""

import asyncio
import inspect
import json
import os
import threading
import traceback
import types

import pytest

pytest.importorskip('fastapi')
pytest.importorskip('httpx')

from fastapi.testclient import TestClient  # noqa: E402, I100
from mtrb_bringup.web_server import (  # noqa: E402, I100
    AUTH_BLOCK_S, AUTH_MAX_FAILURES, AUTH_MAX_TRACKED_IPS, AUTH_WINDOW_S,
    AuthFailureLimiter, create_app, DEFAULT_TOKEN_FILE, is_placeholder_token, is_weak_token,
    load_token, read_token_file, StateStore, TOKEN_ENV_VAR, TOKEN_FILE_MAX_BYTES,
    TOKEN_MIN_LENGTH, TokenError)

TOKEN = 'secret-token-123'
KEYS = {'mode', 'pose', 'scan_age', 'map', 'plan_len', 'nav', 'battery_pct', 'stamp',
        'dock'}


@pytest.fixture
def client():
    return TestClient(create_app(StateStore(), TOKEN))


def _bearer(value):
    return {'Authorization': value}


# --- authentication -----------------------------------------------------------------

def test_state_without_header_returns_401(client):
    response = client.get('/api/state')
    assert response.status_code == 401
    assert response.json()['error'] == 'unauthorized'
    assert response.headers['www-authenticate'] == 'Bearer'


@pytest.mark.parametrize('header', [
    f'Basic {TOKEN}', TOKEN, 'Bearer', 'Bearer ', 'Bearer secret-token-124',
    'Bearer secret-token-12', 'Bearer secret-token-1234', 'Bearer x'])
def test_state_with_bad_credentials_returns_401(client, header):
    response = client.get('/api/state', headers=_bearer(header))
    assert response.status_code == 401
    assert response.json()['error'] == 'unauthorized'


def test_state_with_non_ascii_token_returns_401_without_exception(client):
    response = client.get('/api/state', headers={'Authorization': 'Bearer ñandú'.encode()})
    assert response.status_code == 401


def test_state_with_correct_token_returns_empty_snapshot(client):
    response = client.get('/api/state', headers=_bearer(f'Bearer {TOKEN}'))
    assert response.status_code == 200
    body = response.json()
    assert set(body) == KEYS
    assert all(value is None for value in body.values())


def test_bearer_scheme_is_case_insensitive(client):
    response = client.get('/api/state', headers=_bearer(f'bearer {TOKEN}'))
    assert response.status_code == 200


def test_state_returns_updated_values():
    store = StateStore(clock=lambda: 42.0)
    store.update(mode='nav', plan_len=3)
    body = TestClient(create_app(store, TOKEN)).get(
        '/api/state', headers=_bearer(f'Bearer {TOKEN}')).json()
    assert body['mode'] == 'nav'
    assert body['plan_len'] == 3
    assert body['stamp'] == 42.0


# --- static / app creation ----------------------------------------------------------

def test_root_without_static_dir_returns_404_not_found(client):
    response = client.get('/')
    assert response.status_code == 404
    assert response.json()['error'] == 'not_found'


def test_root_serves_index_when_static_dir_exists(tmp_path):
    (tmp_path / 'index.html').write_text('<html>hi</html>')
    response = TestClient(create_app(StateStore(), TOKEN, str(tmp_path))).get('/')
    assert response.status_code == 200
    assert 'hi' in response.text


def test_static_dir_missing_file_returns_404(tmp_path):
    response = TestClient(create_app(StateStore(), TOKEN, str(tmp_path))).get('/app.js')
    assert response.status_code == 404


@pytest.mark.parametrize('token', ['', None])
def test_create_app_rejects_empty_token(token):
    with pytest.raises(ValueError):
        create_app(StateStore(), token)


def test_docs_endpoints_are_disabled(client):
    assert client.get('/docs').status_code == 404
    assert client.get('/openapi.json').status_code == 404


# --- token sources: see the "token loading" section further down -----------------------

def test_is_placeholder_token():
    assert is_placeholder_token('CHANGE-ME-please')
    assert not is_placeholder_token('real-secret')


# --- StateStore ---------------------------------------------------------------------

def test_store_update_unknown_key_raises_key_error():
    store = StateStore()
    with pytest.raises(KeyError):
        store.update(bogus=1)
    assert store.snapshot()['stamp'] is None


def test_store_update_stamp_is_not_settable():
    with pytest.raises(KeyError):
        StateStore().update(stamp=1.0)


def test_store_snapshot_does_not_share_references():
    store = StateStore()
    store.update(pose={'x': 1.0})
    first = store.snapshot()
    first['pose']['x'] = 99.0
    assert store.snapshot()['pose'] == {'x': 1.0}


def test_store_update_does_not_keep_caller_reference():
    store = StateStore()
    value = {'x': 1.0}
    store.update(pose=value)
    value['x'] = 5.0
    assert store.snapshot()['pose'] == {'x': 1.0}


def test_store_update_refreshes_stamp_with_clock():
    ticks = iter([1.0, 2.0])
    store = StateStore(clock=lambda: next(ticks))
    store.update(mode='a')
    store.update(mode='b')
    assert store.snapshot()['stamp'] == 2.0


def test_store_concurrent_updates_keep_snapshots_consistent():
    store = StateStore()
    errors = []

    def writer(n):
        for i in range(200):
            store.update(pose={'a': n * 1000 + i, 'b': n * 1000 + i})

    def reader():
        for _ in range(200):
            pose = store.snapshot()['pose']
            if pose is not None and pose['a'] != pose['b']:
                errors.append(pose)

    threads = [threading.Thread(target=writer, args=(n,)) for n in range(4)]
    threads += [threading.Thread(target=reader) for _ in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert errors == []


# --- WebSocket /ws (phase 2, I-1/I-2) -------------------------------------------------

def _ws_app(hub=None, max_ws_bytes=4096):
    from mtrb_bringup.web_server import BroadcastHub
    hub = BroadcastHub() if hub is None else hub
    return create_app(StateStore(), TOKEN, hub=hub, max_ws_bytes=max_ws_bytes), hub


def _expect_closed(client, url, code=None):
    from starlette.websockets import WebSocketDisconnect
    with pytest.raises(WebSocketDisconnect) as info:
        with client.websocket_connect(url) as ws:
            ws.receive_text()
    if code is not None:
        assert info.value.code == code
    return info.value.code


@pytest.mark.parametrize('query', ['', '?token=', '?token=wrong', '?token=secret-token-12'])
def test_ws_rejects_missing_empty_or_bad_token(query):
    app, hub = _ws_app()
    _expect_closed(TestClient(app), '/ws' + query)
    assert hub.client_count == 0


def test_ws_accepts_valid_token_and_receives_published_message():
    app, hub = _ws_app()
    with TestClient(app) as client, client.websocket_connect(f'/ws?token={TOKEN}') as ws:
        hub.publish('status', '{"type":"status"}')
        assert ws.receive_text() == '{"type":"status"}'


def test_ws_sixth_client_closed_with_1013_and_slot_freed_on_disconnect():
    from mtrb_bringup.web_server import BroadcastHub
    hub = BroadcastHub(max_clients=5)
    removed = threading.Event()
    original_remove = hub.remove_client

    def remove_and_signal(channel):
        original_remove(channel)
        removed.set()

    hub.remove_client = remove_and_signal
    app, _ = _ws_app(hub)
    url = f'/ws?token={TOKEN}'
    with TestClient(app) as client:
        sockets = [client.websocket_connect(url) for _ in range(5)]
        opened = [s.__enter__() for s in sockets]
        try:
            assert hub.client_count == 5
            _expect_closed(client, url, 1013)
            assert hub.client_count == 5
        finally:
            sockets[0].__exit__(None, None, None)
        assert removed.wait(timeout=3), 'server did not release the client slot'
        assert hub.client_count == 4
        with client.websocket_connect(url) as again:
            hub.publish('status', 'hello')
            assert again.receive_text() == 'hello'
        for sock in sockets[1:]:
            sock.__exit__(None, None, None)
        assert opened


def test_ws_oversized_message_closes_with_1009():
    app, hub = _ws_app(max_ws_bytes=4096)
    with TestClient(app) as client, client.websocket_connect(f'/ws?token={TOKEN}') as ws:
        ws.send_text('x' * 4097)
        from starlette.websockets import WebSocketDisconnect
        with pytest.raises(WebSocketDisconnect) as info:
            ws.receive_text()
        assert info.value.code == 1009


def test_ws_small_message_gets_unsupported_message_error():
    app, hub = _ws_app()
    with TestClient(app) as client, client.websocket_connect(f'/ws?token={TOKEN}') as ws:
        ws.send_text('{"type":"hello"}')
        reply = json.loads(ws.receive_text())
    assert reply['type'] == 'error'
    assert reply['code'] == 'unsupported_message'


def test_ws_map_is_sent_on_connect_meta_then_binary_with_matching_version():
    import struct
    import zlib
    from mtrb_bringup.web_server import json_message
    from mtrb_bringup.web_utils import occupancy_to_bytes, pack_map_binary
    app, hub = _ws_app()
    width, height, version = 3, 2, 7
    meta = json_message('map_meta', version=version, width=width, height=height,
                        resolution=0.05, origin_x=0.0, origin_y=0.0)
    binary = pack_map_binary(version, occupancy_to_bytes([-1, 0, 100] * 2, width, height))
    hub.publish('map', (meta, binary))
    with TestClient(app) as client, client.websocket_connect(f'/ws?token={TOKEN}') as ws:
        first = json.loads(ws.receive_text())
        data = ws.receive_bytes()
    assert first['type'] == 'map_meta'
    assert struct.unpack('<I', data[:4])[0] == first['version'] == version
    assert len(zlib.decompress(data[4:])) == first['width'] * first['height']


def test_ws_publish_from_another_thread_reaches_client():
    app, hub = _ws_app()
    with TestClient(app) as client, client.websocket_connect(f'/ws?token={TOKEN}') as ws:
        thread = threading.Thread(target=hub.publish, args=('pose', 'from-thread'))
        thread.start()
        thread.join()
        assert ws.receive_text() == 'from-thread'


# --- navigation REST API (task 15, U-4) -------------------------------------------------

from mtrb_bringup.web_server import NavBusyError, NavUnavailableError  # noqa: E402, I100

AUTH = {'Authorization': f'Bearer {TOKEN}'}
GOOD = {'x': 3.5, 'y': 2.5, 'yaw': 1.0}
DOCK = (1.5, 1.5, 0.25)


def _grid(**changes):
    """10x10 map, 1 m cells, origin (0, 0). Cell (col 1,row 0) unknown; (2,0) and (9,4) busy."""
    data = [0] * 100
    data[1] = -1
    data[2] = 100
    data[4 * 10 + 9] = 50
    info = {'width': 10, 'height': 10, 'resolution': 1.0, 'origin_x': 0.0, 'origin_y': 0.0,
            'data': data}
    info.update(changes)
    return info


class FakeNav:
    """NavPort double: records the calls and can be told to fail."""

    def __init__(self, goal_error=None, cancel_error=None, pose_error=None):
        self.goals, self.cancels, self.poses = [], 0, []
        self.goal_error, self.cancel_error, self.pose_error = (
            goal_error, cancel_error, pose_error)

    def send_goal(self, x_m, y_m, yaw_rad):
        if self.goal_error:
            raise self.goal_error
        self.goals.append((x_m, y_m, yaw_rad))
        return f'goal-{len(self.goals)}'

    def cancel(self):
        if self.cancel_error:
            raise self.cancel_error
        self.cancels += 1

    def publish_initial_pose(self, x_m, y_m, yaw_rad):
        if self.pose_error:
            raise self.pose_error
        self.poses.append((x_m, y_m, yaw_rad))


def _nav_client(nav='default', dock=DOCK, grid='default', mode='map'):
    store = StateStore()
    store.set_grid(_grid() if grid == 'default' else grid)
    if mode is not None:
        store.update(mode=mode)
    nav = FakeNav() if nav == 'default' else nav
    return TestClient(create_app(store, TOKEN, nav_port=nav, dock=dock)), nav, store


def _post(client, path, body=None, headers=AUTH, raw=None):
    if raw is not None:
        return client.post(path, content=raw, headers={**headers,
                                                       'Content-Type': 'application/json'})
    return client.post(path, json=body, headers=headers)


# store grid

def test_store_grid_is_none_until_set_and_returns_latest():
    store = StateStore()
    assert store.grid() is None
    first, second = _grid(), _grid(width=5)
    store.set_grid(first)
    store.set_grid(second)
    assert store.grid() is second


def test_store_grid_is_not_part_of_the_snapshot():
    store = StateStore()
    store.set_grid(_grid())
    assert 'grid' not in store.snapshot()
    assert set(store.snapshot()) == KEYS


def test_store_set_grid_none_clears_it():
    store = StateStore()
    store.set_grid(_grid())
    store.set_grid(None)
    assert store.grid() is None


# authentication (also with invalid body: 401 wins over 400)

NAV_POSTS = ['/api/goal', '/api/cancel', '/api/dock', '/api/initialpose']


@pytest.mark.parametrize('path', NAV_POSTS)
@pytest.mark.parametrize('headers', [{}, {'Authorization': 'Bearer wrong'},
                                     {'Authorization': TOKEN}])
def test_nav_endpoints_without_valid_token_return_401_and_do_nothing(path, headers):
    client, nav, _ = _nav_client()
    response = _post(client, path, GOOD, headers=headers)
    assert response.status_code == 401
    assert response.json()['error'] == 'unauthorized'
    assert (nav.goals, nav.cancels, nav.poses) == ([], 0, [])


@pytest.mark.parametrize('path', ['/api/goal', '/api/initialpose'])
@pytest.mark.parametrize('raw', [b'{broken', b'[]', b'', b'x' * 5000])
def test_bad_token_with_invalid_body_still_returns_401(path, raw):
    client, nav, _ = _nav_client()
    response = _post(client, path, headers={'Authorization': 'Bearer wrong'}, raw=raw)
    assert response.status_code == 401
    assert nav.goals == [] and nav.poses == []


def test_missing_token_with_invalid_body_returns_401():
    client, _, _ = _nav_client()
    assert _post(client, '/api/goal', headers={}, raw=b'{broken').status_code == 401


# body validation (400 invalid_body / invalid_json / body_too_large)

@pytest.mark.parametrize('path', ['/api/goal', '/api/initialpose'])
@pytest.mark.parametrize('body', [
    {'x': True, 'y': 2.5, 'yaw': 1.0},
    {'x': 3.5, 'y': False, 'yaw': 1.0},
    {'x': 3.5, 'y': 2.5, 'yaw': True},
    {'x': '3.5', 'y': 2.5, 'yaw': 1.0},
    {'x': None, 'y': 2.5, 'yaw': 1.0},
    {'x': [3.5], 'y': 2.5, 'yaw': 1.0},
    {'y': 2.5, 'yaw': 1.0},
    {'x': 3.5, 'y': 2.5},
    {},
    {'x': 3.5, 'y': 2.5, 'yaw': 1.0, 'extra': 1},
    [], [3.5, 2.5, 1.0], 'text', 7, None, True,
    {'x': 10001, 'y': 2.5, 'yaw': 1.0},
    {'x': 3.5, 'y': -1e300, 'yaw': 1.0},
    {'x': 3.5, 'y': 2.5, 'yaw': 1e9},
])
def test_invalid_body_returns_400_invalid_body_and_calls_nothing(path, body):
    client, nav, _ = _nav_client()
    response = _post(client, path, body)
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_body'
    assert (nav.goals, nav.poses) == ([], [])


@pytest.mark.parametrize('path', ['/api/goal', '/api/initialpose'])
@pytest.mark.parametrize('literal', ['NaN', 'Infinity', '-Infinity'])
@pytest.mark.parametrize('field', ['x', 'y', 'yaw'])
def test_non_finite_literal_returns_400_invalid_json(path, literal, field):
    client, nav, _ = _nav_client()
    values = {'x': '3.5', 'y': '2.5', 'yaw': '1.0', field: literal}
    raw = f'{{"x": {values["x"]}, "y": {values["y"]}, "yaw": {values["yaw"]}}}'.encode()
    response = _post(client, path, raw=raw)
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_json'
    assert (nav.goals, nav.poses) == ([], [])


@pytest.mark.parametrize('path', ['/api/goal', '/api/initialpose'])
@pytest.mark.parametrize('raw', [b'{broken', b'{"x": 1,', b'\xff\xfe', b'not json',
                                 b'[' * 1000])
def test_broken_json_returns_400_invalid_json(path, raw):
    client, nav, _ = _nav_client()
    response = _post(client, path, raw=raw)
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_json'
    assert (nav.goals, nav.poses) == ([], [])


@pytest.mark.parametrize('path', ['/api/goal', '/api/initialpose'])
def test_empty_body_returns_400_invalid_body(path):
    client, nav, _ = _nav_client()
    response = _post(client, path, raw=b'')
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_body'


@pytest.mark.parametrize('path', ['/api/goal', '/api/initialpose'])
def test_body_over_1kb_returns_400_body_too_large(path):
    client, nav, _ = _nav_client()
    padding = ' ' * 1100
    raw = ('{"x": 3.5, "y": 2.5, "yaw": 1.0}' + padding).encode()
    response = _post(client, path, raw=raw)
    assert response.status_code == 400
    assert response.json()['error'] == 'body_too_large'
    assert (nav.goals, nav.poses) == ([], [])


def test_chunked_body_over_1kb_without_content_length_returns_400():
    client, nav, _ = _nav_client()

    def chunks():
        yield b'{"x": 3.5, "y": 2.5, "yaw": 1.0,'
        for _ in range(20):
            yield b' ' * 100
        yield b'}'

    response = client.post('/api/goal', content=chunks(), headers={
        **AUTH, 'Content-Type': 'application/json'})
    assert response.status_code == 400
    assert response.json()['error'] == 'body_too_large'
    assert nav.goals == []


def test_body_of_exactly_1kb_is_not_refused_as_too_large():
    client, nav, _ = _nav_client()
    head = b'{"x": 3.5, "y": 2.5, "yaw": 1.0}'
    raw = head + b' ' * (1024 - len(head))
    assert len(raw) == 1024
    response = _post(client, '/api/goal', raw=raw)
    assert response.status_code == 200
    assert len(nav.goals) == 1


# /api/goal

def test_goal_ok_returns_goal_id_and_double_gets_the_validated_floats():
    client, nav, _ = _nav_client()
    response = _post(client, '/api/goal', {'x': 3, 'y': 2, 'yaw': 1})
    assert response.status_code == 200
    assert response.json() == {'ok': True, 'goal_id': 'goal-1'}
    assert nav.goals == [(3.0, 2.0, 1.0)]
    assert all(type(v) is float for v in nav.goals[0])


@pytest.mark.parametrize('x, y', [(-0.5, 2.5), (10.0, 2.5), (3.5, -0.01), (3.5, 10.5),
                                  (10000.0, 10000.0), (-10000.0, 0.0)])
def test_goal_outside_map_returns_400_invalid_goal(x, y):
    client, nav, _ = _nav_client()
    response = _post(client, '/api/goal', {'x': x, 'y': y, 'yaw': 0})
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_goal'
    assert nav.goals == []


@pytest.mark.parametrize('x, y', [(2.5, 0.5), (9.5, 4.5)])
def test_goal_on_occupied_cell_ge_50_returns_400_invalid_goal(x, y):
    client, nav, _ = _nav_client()
    response = _post(client, '/api/goal', {'x': x, 'y': y, 'yaw': 0})
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_goal'
    assert nav.goals == []


def test_goal_on_unknown_cell_minus_one_is_accepted_d3():
    client, nav, _ = _nav_client()
    response = _post(client, '/api/goal', {'x': 1.5, 'y': 0.5, 'yaw': 0})
    assert response.status_code == 200
    assert nav.goals == [(1.5, 0.5, 0.0)]


def test_goal_on_free_cell_just_below_threshold_is_accepted():
    data = [0] * 100
    data[0] = 49
    client, nav, _ = _nav_client(grid=_grid(data=data))
    assert _post(client, '/api/goal', {'x': 0.5, 'y': 0.5, 'yaw': 0}).status_code == 200


def test_goal_without_map_returns_503_no_map():
    client, nav, _ = _nav_client(grid=None)
    response = _post(client, '/api/goal', GOOD)
    assert response.status_code == 503
    assert response.json()['error'] == 'no_map'
    assert nav.goals == []


def test_goal_with_cleared_map_returns_503_no_map():
    client, nav, store = _nav_client()
    store.set_grid(None)
    assert _post(client, '/api/goal', GOOD).json()['error'] == 'no_map'


def test_goal_without_nav_port_returns_503_nav_unavailable():
    client, _, _ = _nav_client(nav=None)
    response = _post(client, '/api/goal', GOOD)
    assert response.status_code == 503
    assert response.json()['error'] == 'nav_unavailable'


def test_goal_when_nav2_unavailable_returns_503_nav_unavailable():
    client, nav, _ = _nav_client(nav=FakeNav(goal_error=NavUnavailableError('no server')))
    response = _post(client, '/api/goal', GOOD)
    assert response.status_code == 503
    assert response.json()['error'] == 'nav_unavailable'


def test_goal_while_another_is_active_returns_409_goal_active():
    client, nav, _ = _nav_client(nav=FakeNav(goal_error=NavBusyError('busy')))
    response = _post(client, '/api/goal', GOOD)
    assert response.status_code == 409
    assert response.json()['error'] == 'goal_active'


def test_goal_busy_detail_for_an_active_goal_is_reported_to_the_client():
    message = 'A goal is already active: cancel it first'
    client, _, _ = _nav_client(nav=FakeNav(goal_error=NavBusyError(message)))
    response = _post(client, '/api/goal', GOOD)
    assert response.status_code == 409
    assert response.json() == {'error': 'goal_active', 'detail': message}


def test_goal_busy_detail_for_unconfirmed_orphans_is_reported_to_the_client():
    message = 'A previous goal has not been confirmed finished by Nav2; cancel and wait'
    client, _, _ = _nav_client(nav=FakeNav(goal_error=NavBusyError(message)))
    response = _post(client, '/api/goal', GOOD)
    assert response.status_code == 409
    assert response.json() == {'error': 'goal_active', 'detail': message}


def test_goal_busy_without_message_uses_the_default_detail():
    client, _, _ = _nav_client(nav=FakeNav(goal_error=NavBusyError()))
    response = _post(client, '/api/goal', GOOD)
    assert response.status_code == 409
    assert response.json()['error'] == 'goal_active'
    assert response.json()['detail'] == 'A goal is already active'


def test_state_returns_the_dock_pose_when_create_app_receives_it():
    client, _, _ = _nav_client(dock=(1.5, -2.0, 0.25))
    body = client.get('/api/state', headers=AUTH).json()
    assert body['dock'] == {'x': 1.5, 'y': -2.0, 'yaw': 0.25}


def test_state_dock_is_null_when_create_app_has_no_dock():
    client, _, _ = _nav_client(dock=None)
    body = client.get('/api/state', headers=AUTH).json()
    assert 'dock' in body
    assert body['dock'] is None


def test_goal_invalid_body_is_checked_before_missing_nav_port_or_map():
    client, _, _ = _nav_client(nav=None, grid=None)
    assert _post(client, '/api/goal', {'x': True, 'y': 1, 'yaw': 0}).status_code == 400


# /api/cancel

def test_cancel_calls_the_port_once_and_returns_ok():
    client, nav, _ = _nav_client()
    response = _post(client, '/api/cancel')
    assert response.status_code == 200
    assert response.json() == {'ok': True}
    assert nav.cancels == 1
    assert nav.goals == [] and nav.poses == []


def test_cancel_is_idempotent_with_nothing_active():
    client, nav, _ = _nav_client()
    for _ in range(3):
        response = _post(client, '/api/cancel')
        assert response.status_code == 200
        assert response.json() == {'ok': True}
    assert nav.cancels == 3


def test_cancel_ignores_body_and_works_without_map():
    client, nav, _ = _nav_client(grid=None)
    response = _post(client, '/api/cancel', raw=b'{broken')
    assert response.status_code == 200
    assert nav.cancels == 1


def test_cancel_without_nav_port_returns_503():
    client, _, _ = _nav_client(nav=None)
    response = _post(client, '/api/cancel')
    assert response.status_code == 503
    assert response.json()['error'] == 'nav_unavailable'


def test_cancel_when_nav2_unavailable_returns_503():
    client, _, _ = _nav_client(nav=FakeNav(cancel_error=NavUnavailableError('x')))
    assert _post(client, '/api/cancel').status_code == 503


# /api/dock

def test_dock_with_map_sends_the_configured_pose_as_floats():
    client, nav, _ = _nav_client()
    response = _post(client, '/api/dock')
    assert response.status_code == 200
    assert response.json() == {'ok': True, 'goal_id': 'goal-1'}
    assert nav.goals == [DOCK]


def test_dock_ignores_the_client_body():
    client, nav, _ = _nav_client()
    response = _post(client, '/api/dock', {'x': 9.0, 'y': 9.0, 'yaw': 9.0})
    assert response.status_code == 200
    assert nav.goals == [DOCK]


def test_dock_without_map_is_sent_unchecked():
    client, nav, _ = _nav_client(grid=None)
    response = _post(client, '/api/dock')
    assert response.status_code == 200
    assert nav.goals == [DOCK]


def test_dock_outside_the_map_returns_400_invalid_goal():
    client, nav, _ = _nav_client(dock=(50.0, 50.0, 0.0))
    response = _post(client, '/api/dock')
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_goal'
    assert nav.goals == []


def test_dock_not_configured_returns_503_dock_unavailable():
    client, nav, _ = _nav_client(dock=None)
    response = _post(client, '/api/dock')
    assert response.status_code == 503
    assert response.json()['error'] == 'dock_unavailable'
    assert nav.goals == []


def test_dock_without_nav_port_returns_503_nav_unavailable():
    client, _, _ = _nav_client(nav=None)
    response = _post(client, '/api/dock')
    assert response.status_code == 503
    assert response.json()['error'] == 'nav_unavailable'


def test_dock_while_goal_active_returns_409_goal_active_with_and_without_map():
    for grid in ('default', None):
        client, _, _ = _nav_client(nav=FakeNav(goal_error=NavBusyError('busy')), grid=grid)
        response = _post(client, '/api/dock')
        assert response.status_code == 409
        assert response.json()['error'] == 'goal_active'


def test_dock_when_nav2_unavailable_returns_503():
    client, _, _ = _nav_client(nav=FakeNav(goal_error=NavUnavailableError('x')))
    assert _post(client, '/api/dock').status_code == 503


# /api/initialpose

def test_initialpose_in_map_mode_publishes_validated_floats_and_never_sends_a_goal():
    client, nav, _ = _nav_client(mode='map')
    response = _post(client, '/api/initialpose', {'x': 1, 'y': 2, 'yaw': 3})
    assert response.status_code == 200
    assert response.json() == {'ok': True}
    assert nav.poses == [(1.0, 2.0, 3.0)]
    assert all(type(v) is float for v in nav.poses[0])
    assert nav.goals == []


def test_initialpose_without_a_map_returns_503_no_map_and_publishes_nothing():
    client, nav, _ = _nav_client(grid=None, mode='map')
    response = _post(client, '/api/initialpose', GOOD)
    assert response.status_code == 503
    assert response.json()['error'] == 'no_map'
    assert nav.poses == []


@pytest.mark.parametrize('pose', [{'x': 500, 'y': 500, 'yaw': 0}, {'x': -0.5, 'y': 1, 'yaw': 0},
                                  {'x': 10.0, 'y': 1, 'yaw': 0}, {'x': 1, 'y': 10.5, 'yaw': 0},
                                  {'x': 1, 'y': -1, 'yaw': 0}])
def test_initialpose_outside_the_map_returns_400_invalid_pose(pose):
    client, nav, _ = _nav_client(mode='map')
    response = _post(client, '/api/initialpose', pose)
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_pose'
    assert 'Goal' not in response.json()['detail']
    assert nav.poses == []


@pytest.mark.parametrize('pose', [
    {'x': 1.5, 'y': 0.5, 'yaw': 0},     # cell (1, 0): unknown
    {'x': 2.5, 'y': 0.5, 'yaw': 0},     # cell (2, 0): occupied
    {'x': 9.5, 'y': 4.5, 'yaw': 0}])    # cell (9, 4): occupied (50)
def test_initialpose_on_an_occupied_or_unknown_cell_inside_the_map_is_allowed(pose):
    client, nav, _ = _nav_client(mode='map')
    assert _post(client, '/api/initialpose', pose).status_code == 200
    assert nav.poses == [(pose['x'], pose['y'], 0.0)]


def test_initialpose_check_order_body_then_mode_then_map_then_pose_then_busy():
    # 1) body first: invalid body wins over a wrong mode
    client, nav, _ = _nav_client(mode='mapping', grid=None)
    assert _post(client, '/api/initialpose', {'x': 1}).status_code == 400
    # 2) mode before map: wrong mode and no map -> 409
    assert _post(client, '/api/initialpose', GOOD).status_code == 409
    # 3) map before pose: right mode, no map, pose outside the map -> 503
    client, nav, _ = _nav_client(mode='map', grid=None)
    assert _post(client, '/api/initialpose', {'x': 500, 'y': 500, 'yaw': 0}).status_code == 503
    # 4) pose before busy: the port is never reached for an outside pose
    busy = FakeNav(pose_error=NavBusyError('busy'))
    client, nav, _ = _nav_client(nav=busy, mode='map')
    assert _post(client, '/api/initialpose', {'x': 500, 'y': 500, 'yaw': 0}).status_code == 400
    # 5) busy last
    response = _post(client, '/api/initialpose', GOOD)
    assert response.status_code == 409
    assert response.json()['error'] == 'goal_active'


def test_initialpose_while_nav_is_busy_returns_409_goal_active_and_publishes_nothing():
    client, nav, _ = _nav_client(nav=FakeNav(pose_error=NavBusyError('busy')), mode='map')
    response = _post(client, '/api/initialpose', GOOD)
    assert response.status_code == 409
    assert response.json()['error'] == 'goal_active'
    assert nav.poses == []


def test_initialpose_when_nav_is_unavailable_returns_503():
    client, _, _ = _nav_client(nav=FakeNav(pose_error=NavUnavailableError('x')), mode='map')
    assert _post(client, '/api/initialpose', GOOD).status_code == 503


def test_initialpose_in_mapping_mode_returns_409_wrong_mode():
    client, nav, _ = _nav_client(mode='mapping')
    response = _post(client, '/api/initialpose', GOOD)
    assert response.status_code == 409
    assert response.json()['error'] == 'wrong_mode'
    assert nav.poses == []


def test_initialpose_with_unknown_mode_returns_409_wrong_mode():
    client, nav, _ = _nav_client(mode=None)
    response = _post(client, '/api/initialpose', GOOD)
    assert response.status_code == 409
    assert response.json()['error'] == 'wrong_mode'
    assert nav.poses == []


def test_initialpose_without_nav_port_returns_503():
    client, _, _ = _nav_client(nav=None, mode='map')
    response = _post(client, '/api/initialpose', GOOD)
    assert response.status_code == 503
    assert response.json()['error'] == 'nav_unavailable'


def test_initialpose_invalid_body_in_wrong_mode_is_400():
    client, _, _ = _nav_client(mode='mapping')
    assert _post(client, '/api/initialpose', {'x': 1}).status_code == 400


# cross-cutting

def test_each_command_reaches_exactly_the_matching_port_method():
    client, nav, _ = _nav_client()
    _post(client, '/api/cancel')
    _post(client, '/api/initialpose', GOOD)
    assert (nav.goals, nav.cancels, len(nav.poses)) == ([], 1, 1)
    _post(client, '/api/goal', GOOD)
    assert (len(nav.goals), nav.cancels, len(nav.poses)) == (1, 1, 1)


@pytest.mark.parametrize('path', NAV_POSTS)
def test_nav_endpoints_reject_get(path):
    client, nav, _ = _nav_client()
    assert client.get(path, headers=AUTH).status_code == 405
    assert (nav.goals, nav.cancels, nav.poses) == ([], 0, [])


# --- teleop and stop over WebSocket / REST (task 18, criteria I-3 and A-4) --------------------
# SAFETY: the server only forwards teleop/stop to the NavPort (the node holds the decisions).
# These tests fix WHAT is forwarded, with WHICH client id, and what is never forwarded.

class FakeTeleopNav(FakeNav):
    """NavPort double with the teleop/stop methods; records every call in order."""

    def __init__(self, teleop_reply=None):
        super().__init__()
        self.calls = []                      # ('teleop', id, msg) | ('stop',) | ('disconnect', id)
        self.teleop_reply = teleop_reply     # None or (code, detail) returned by teleop()
        self.disconnected = threading.Event()

    def teleop(self, client_id, message):
        self.calls.append(('teleop', client_id, message))
        return self.teleop_reply

    def teleop_disconnect(self, client_id):
        self.calls.append(('disconnect', client_id))
        self.disconnected.set()

    def stop(self):
        self.calls.append(('stop',))

    def of(self, kind):
        return [c for c in self.calls if c[0] == kind]


WS_URL = f'/ws?token={TOKEN}'
TELEOP = {'type': 'teleop', 'vx': 0.1, 'wz': -0.2, 'seq': 1}


def _teleop_app(nav='default', max_ws_bytes=4096):
    nav = FakeTeleopNav() if nav == 'default' else nav
    return TestClient(create_app(StateStore(), TOKEN, nav_port=nav,
                                 max_ws_bytes=max_ws_bytes)), nav


def _sync(ws):
    """Round trip that proves every earlier message of this socket was dispatched."""
    ws.send_text('{"type":"sync-probe"}')
    reply = json.loads(ws.receive_text())
    assert reply['code'] == 'unsupported_message'


def test_nav_port_base_class_teleop_methods_are_abstract():
    from mtrb_bringup.web_server import NavPort
    port = NavPort()
    with pytest.raises(NotImplementedError):
        port.teleop(1, {})
    with pytest.raises(NotImplementedError):
        port.teleop_disconnect(1)
    with pytest.raises(NotImplementedError):
        port.stop()


# POST /api/stop

def test_stop_without_token_returns_401_and_does_not_stop():
    client, nav = _teleop_app()
    response = client.post('/api/stop')
    assert response.status_code == 401
    assert nav.calls == []


@pytest.mark.parametrize('header', ['Bearer wrong', 'Bearer ', f'Basic {TOKEN}', TOKEN])
def test_stop_with_bad_token_returns_401_and_does_not_stop(header):
    client, nav = _teleop_app()
    assert client.post('/api/stop', headers=_bearer(header)).status_code == 401
    assert nav.calls == []


def test_stop_with_token_returns_ok_and_stops_exactly_once():
    client, nav = _teleop_app()
    response = client.post('/api/stop', headers=AUTH)
    assert response.status_code == 200
    assert response.json() == {'ok': True}
    assert nav.calls == [('stop',)]


def test_stop_does_not_cancel_directly_nor_send_goals():
    client, nav = _teleop_app()
    client.post('/api/stop', headers=AUTH)
    assert (nav.cancels, nav.goals, nav.poses) == (0, [], [])


def test_stop_without_nav_port_returns_503_nav_unavailable():
    client = TestClient(create_app(StateStore(), TOKEN))
    response = client.post('/api/stop', headers=AUTH)
    assert response.status_code == 503
    assert response.json()['error'] == 'nav_unavailable'


@pytest.mark.parametrize('body', [None, {'x': 'garbage'}, {}, [1, 2, 3]])
def test_stop_ignores_any_body(body):
    client, nav = _teleop_app()
    assert _post(client, '/api/stop', body).status_code == 200
    assert nav.calls == [('stop',)]


def test_stop_ignores_malformed_json_and_oversized_bodies():
    client, nav = _teleop_app()
    assert _post(client, '/api/stop', raw='{not json').status_code == 200
    assert _post(client, '/api/stop', raw='x' * 5000).status_code == 200
    assert nav.calls == [('stop',), ('stop',)]


def test_stop_does_not_need_map_mode_or_a_known_state():
    store = StateStore()  # no map, no mode
    nav = FakeTeleopNav()
    client = TestClient(create_app(store, TOKEN, nav_port=nav))
    assert client.post('/api/stop', headers=AUTH).status_code == 200
    assert nav.calls == [('stop',)]


def test_stop_in_mapping_mode_is_accepted():
    client, nav, _ = _nav_client(nav=FakeTeleopNav(), mode='mapping', grid=None)
    assert client.post('/api/stop', headers=AUTH).status_code == 200
    assert nav.calls == [('stop',)]


def test_stop_is_repeatable_every_call_reaches_the_port():
    client, nav = _teleop_app()
    for _ in range(10):
        assert client.post('/api/stop', headers=AUTH).status_code == 200
    assert nav.calls == [('stop',)] * 10


def test_stop_rejects_get():
    client, nav = _teleop_app()
    assert client.get('/api/stop', headers=AUTH).status_code == 405
    assert nav.calls == []


# WebSocket stop

def test_ws_stop_calls_the_port_and_sends_no_reply():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text('{"type":"stop"}')
        _sync(ws)
    assert nav.of('stop') == [('stop',)]


def test_ws_stop_is_not_rate_limited_by_the_server():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        for _ in range(60):
            ws.send_text('{"type":"stop"}')
        _sync(ws)
    assert len(nav.of('stop')) == 60


def test_ws_stop_from_another_client_is_dispatched_between_the_owners_teleops():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as owner, \
            client.websocket_connect(WS_URL) as other:
        owner.send_text(json.dumps(TELEOP))
        other.send_text('{"type":"stop"}')
        _sync(other)
        owner.send_text(json.dumps({**TELEOP, 'seq': 2}))
        _sync(owner)
    kinds = [c[0] for c in nav.calls if c[0] != 'disconnect']
    assert kinds.count('stop') == 1
    assert kinds.count('teleop') == 2


def test_ws_stop_ignores_extra_fields_and_never_reaches_teleop():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text('{"type":"stop","vx":1,"seq":-5,"junk":[1,2]}')
        _sync(ws)
    assert nav.of('stop') == [('stop',)]
    assert nav.of('teleop') == []


def test_ws_stop_without_nav_port_answers_stop_unavailable():
    client = TestClient(create_app(StateStore(), TOKEN))
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text('{"type":"stop"}')
        reply = json.loads(ws.receive_text())
    assert reply['type'] == 'error'
    assert reply['code'] == 'stop_unavailable'
    assert 'physical stop' in reply['detail']


def test_ws_teleop_without_nav_port_is_rejected():
    client = TestClient(create_app(StateStore(), TOKEN))
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(json.dumps(TELEOP))
        reply = json.loads(ws.receive_text())
    assert reply['type'] == 'error'
    assert reply['code'] == 'teleop_rejected'


# WebSocket teleop dispatch

def test_ws_teleop_is_forwarded_to_the_port_with_the_whole_message():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(json.dumps(TELEOP))
        _sync(ws)
    calls = nav.of('teleop')
    assert len(calls) == 1
    assert calls[0][2] == TELEOP
    assert nav.of('stop') == []


def test_ws_teleop_accepted_gets_no_reply():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(json.dumps(TELEOP))
        _sync(ws)  # the first reply is the probe's: teleop produced none


def test_ws_teleop_error_of_the_port_is_sent_to_that_client_only():
    nav = FakeTeleopNav(teleop_reply=('teleop_rejected', 'Another client is controlling'))
    client, _ = _teleop_app(nav)
    with client, client.websocket_connect(WS_URL) as sender, \
            client.websocket_connect(WS_URL) as bystander:
        sender.send_text(json.dumps(TELEOP))
        reply = json.loads(sender.receive_text())
        _sync(bystander)  # the bystander only gets its own probe reply
    assert reply == {'type': 'error', 'code': 'teleop_rejected',
                     'detail': 'Another client is controlling'}


def test_ws_teleop_messages_reach_the_port_in_order():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        for seq in range(1, 6):
            ws.send_text(json.dumps({**TELEOP, 'seq': seq}))
        _sync(ws)
    assert [c[2]['seq'] for c in nav.of('teleop')] == [1, 2, 3, 4, 5]


def test_ws_stop_after_teleop_keeps_the_order_teleop_then_stop():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(json.dumps(TELEOP))
        ws.send_text('{"type":"stop"}')
        _sync(ws)
    assert [c[0] for c in nav.calls if c[0] != 'disconnect'] == ['teleop', 'stop']


def test_ws_teleop_and_stop_use_the_same_client_id_as_the_disconnect():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(json.dumps(TELEOP))
        _sync(ws)
    assert nav.disconnected.wait(timeout=3), 'teleop_disconnect was not called'
    assert nav.of('teleop')[0][1] == nav.of('disconnect')[0][1]


def test_each_connection_gets_a_different_client_id():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as first, \
            client.websocket_connect(WS_URL) as second:
        first.send_text(json.dumps(TELEOP))
        _sync(first)
        second.send_text(json.dumps(TELEOP))
        _sync(second)
    ids = [c[1] for c in nav.of('teleop')]
    assert len(ids) == 2 and ids[0] != ids[1]


def test_client_ids_are_not_reused_by_later_connections():
    client, nav = _teleop_app()
    with client:
        for _ in range(3):
            with client.websocket_connect(WS_URL) as ws:
                ws.send_text(json.dumps(TELEOP))
                _sync(ws)
    ids = [c[1] for c in nav.of('teleop')]
    assert len(set(ids)) == 3


def test_client_id_is_never_taken_from_the_message():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(json.dumps({**TELEOP, 'client_id': 'spoofed', 'id': 'spoofed'}))
        _sync(ws)
    client_id = nav.of('teleop')[0][1]
    assert client_id != 'spoofed' and isinstance(client_id, int)


# WebSocket close -> teleop_disconnect

def test_closing_the_websocket_calls_teleop_disconnect_once():
    client, nav = _teleop_app()
    with client:
        with client.websocket_connect(WS_URL) as ws:
            ws.send_text(json.dumps(TELEOP))
            _sync(ws)
        assert nav.disconnected.wait(timeout=3), 'teleop_disconnect was not called'
    assert len(nav.of('disconnect')) == 1


def test_a_connection_that_never_sent_teleop_also_reports_its_disconnect():
    client, nav = _teleop_app()
    with client:
        with client.websocket_connect(WS_URL):
            pass
        assert nav.disconnected.wait(timeout=3)
    assert nav.of('teleop') == []
    assert len(nav.of('disconnect')) == 1


def test_oversized_message_closes_with_1009_and_still_reports_the_disconnect():
    client, nav = _teleop_app()
    from starlette.websockets import WebSocketDisconnect
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(json.dumps({**TELEOP, 'pad': 'x' * 5000}))
        with pytest.raises(WebSocketDisconnect) as info:
            ws.receive_text()
    assert info.value.code == 1009
    assert nav.disconnected.wait(timeout=3)
    assert nav.of('teleop') == []


def test_rejected_token_never_reaches_the_port():
    client, nav = _teleop_app()
    _expect_closed(client, '/ws?token=wrong')
    assert nav.calls == []


# size limit, invalid JSON

def _padded_teleop(total_bytes):
    base = json.dumps({**TELEOP, 'pad': ''}, separators=(',', ':'))
    return json.dumps({**TELEOP, 'pad': 'x' * (total_bytes - len(base))},
                      separators=(',', ':'))


def test_message_of_exactly_4096_bytes_is_still_dispatched():
    text = _padded_teleop(4096)
    assert len(text.encode()) == 4096
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(text)
        _sync(ws)
    assert len(nav.of('teleop')) == 1


def test_message_of_4097_bytes_closes_with_1009_without_dispatch():
    text = _padded_teleop(4097)
    assert len(text.encode()) == 4097
    client, nav = _teleop_app()
    from starlette.websockets import WebSocketDisconnect
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(text)
        with pytest.raises(WebSocketDisconnect) as info:
            ws.receive_text()
    assert info.value.code == 1009
    assert nav.of('teleop') == []


def test_multibyte_text_is_measured_in_bytes_not_characters():
    # ~2100 characters but more than 4096 bytes in UTF-8
    text = json.dumps({**TELEOP, 'pad': 'ñ' * 2100}, ensure_ascii=False)
    assert len(text) < 4096 < len(text.encode())
    client, nav = _teleop_app()
    from starlette.websockets import WebSocketDisconnect
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(text)
        with pytest.raises(WebSocketDisconnect) as info:
            ws.receive_text()
    assert info.value.code == 1009


def test_binary_message_larger_than_the_limit_closes_with_1009():
    client, nav = _teleop_app()
    from starlette.websockets import WebSocketDisconnect
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_bytes(b'\x00' * 4097)
        with pytest.raises(WebSocketDisconnect) as info:
            ws.receive_text()
    assert info.value.code == 1009
    assert nav.of('stop') == [] and nav.of('teleop') == []


@pytest.mark.parametrize('raw', [
    '{"type":"teleop","vx":NaN,"wz":0,"seq":1}',
    '{"type":"teleop","vx":0,"wz":Infinity,"seq":1}',
    '{"type":"teleop","vx":0,"wz":-Infinity,"seq":1}',
    '{"type":"teleop","vx":0,"wz":0,"seq":NaN}',
    '{not json', '', '   ', 'null', '[]', '[{"type":"stop"}]', '"stop"', '5', 'true',
    '{"type":"teleop"', '{"type":"stop"}}',
    '{"type":"Stop"}', '{"type":"TELEOP"}', '{"type":" stop"}', '{"type":null}',
    '{"type":["stop"]}', '{"type":1}', '{}', '{"kind":"stop"}'])
def test_ws_invalid_json_or_unknown_type_is_unsupported_message_and_never_dispatched(raw):
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(raw)
        reply = json.loads(ws.receive_text())
    assert reply['type'] == 'error'
    assert reply['code'] == 'unsupported_message'
    assert nav.of('teleop') == [] and nav.of('stop') == []


def test_ws_binary_frame_is_unsupported_message_and_never_dispatched():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_bytes(b'{"type":"stop"}')
        reply = json.loads(ws.receive_text())
    assert reply['code'] == 'unsupported_message'
    assert nav.of('stop') == []


def test_ws_deeply_nested_json_does_not_crash_the_receiver():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text('[' * 1500 + ']' * 1500)
        reply = json.loads(ws.receive_text())
        ws.send_text('{"type":"stop"}')
        _sync(ws)
    assert reply['code'] == 'unsupported_message'
    assert nav.of('stop') == [('stop',)]


def test_connection_keeps_working_after_invalid_messages():
    client, nav = _teleop_app()
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text('{broken')
        _sync(ws)
        ws.send_text('{"type":"stop"}')
        _sync(ws)
    assert nav.of('stop') == [('stop',)]


# the server never logs teleop content

def test_server_source_never_logs():
    import inspect
    import re
    from mtrb_bringup import web_server
    source = inspect.getsource(web_server)
    assert not re.search(r'\bimport logging\b|\bgetLogger\b|\bprint\(|\blogger\b|'
                         r'\bwarnings\.warn\b', source)


def test_dispatch_code_does_not_reference_logging_of_messages():
    import inspect
    from mtrb_bringup import web_server
    for func in (web_server._handle_client_message, web_server._receive_loop,
                 web_server._parse_ws_text):
        body = inspect.getsource(func).lower()
        assert 'logg' not in body.replace('never logged', '')
        assert 'print(' not in body


def test_server_emits_no_log_records_while_handling_teleop_and_stop(caplog):
    import logging
    client, nav = _teleop_app(FakeTeleopNav(teleop_reply=('teleop_rejected', 'x')))
    with caplog.at_level(logging.DEBUG):
        with client, client.websocket_connect(WS_URL) as ws:
            ws.send_text(json.dumps({**TELEOP, 'vx': 0.123456789, 'seq': 987654321}))
            ws.receive_text()
            ws.send_text('{"type":"stop"}')
            _sync(ws)
    ours = [r for r in caplog.records if r.name.startswith('mtrb_bringup')]
    assert ours == []
    assert '0.123456789' not in caplog.text and '987654321' not in caplog.text


# --- phase 4: huge integers and hostile Content-Length (never a 500) -------------------------

HUGE = 10 ** 400


@pytest.mark.parametrize('path', ['/api/goal', '/api/initialpose'])
@pytest.mark.parametrize('field', ['x', 'y', 'yaw'])
@pytest.mark.parametrize('huge', [HUGE, -HUGE])
def test_huge_integer_in_the_pose_returns_400_invalid_body_never_500(path, field, huge):
    client, nav, _ = _nav_client()
    body = {'x': 3.5, 'y': 2.5, 'yaw': 1.0, field: huge}
    response = _post(client, path, raw=json.dumps(body).encode())
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_body'
    assert (nav.goals, nav.poses) == ([], [])


def test_huge_integer_goal_in_a_map_with_a_huge_origin_is_400_not_500():
    # The goal itself is fine; the map metadata overflows: still a clean refusal.
    client, nav, _ = _nav_client(grid=_grid(origin_x=HUGE))
    response = _post(client, '/api/goal', GOOD)
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_goal'
    assert nav.goals == []


def test_dock_with_a_huge_integer_in_the_map_metadata_is_400_not_500():
    client, nav, _ = _nav_client(grid=_grid(resolution=HUGE, width=HUGE))
    response = _post(client, '/api/dock')
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_goal'
    assert nav.goals == []


def test_dock_configured_with_a_huge_integer_is_refused_not_a_500():
    client, nav, _ = _nav_client(dock=(HUGE, 0.0, 0.0))
    response = _post(client, '/api/dock')
    assert response.status_code == 400
    assert response.json()['error'] == 'invalid_goal'
    assert nav.goals == []


def test_the_server_keeps_working_after_a_huge_integer_request():
    client, nav, _ = _nav_client()
    assert _post(client, '/api/goal', raw=json.dumps({'x': HUGE, 'y': 1, 'yaw': 0}).encode(
        )).status_code == 400
    assert _post(client, '/api/goal', GOOD).status_code == 200
    assert len(nav.goals) == 1


def _post_with_content_length(client, declared, body, path='/api/goal'):
    return client.post(path, content=body, headers={
        **AUTH, 'Content-Type': 'application/json', 'Content-Length': declared})


VALID_BODY = b'{"x": 3.5, "y": 2.5, "yaw": 1.0}'
OVERSIZED_BODY = VALID_BODY + b' ' * 2000


@pytest.mark.parametrize('declared', [b'\xb2', b'\xb2\xb2\xb2', b'\xff', b'1\xb2'])
def test_non_ascii_content_length_with_an_oversized_body_returns_400_body_too_large(declared):
    client, nav, _ = _nav_client()
    response = _post_with_content_length(client, declared, OVERSIZED_BODY)
    assert response.status_code == 400
    assert response.json()['error'] == 'body_too_large'
    assert nav.goals == []


@pytest.mark.parametrize('declared', [b'\xb2', b'\xff', b'1\xb2'])
def test_non_ascii_content_length_never_causes_a_500(declared):
    client, nav, _ = _nav_client()
    response = _post_with_content_length(client, declared, VALID_BODY)
    assert response.status_code < 500
    assert response.status_code in (200, 400)


@pytest.mark.parametrize('digits', [4301, 5000, 20000])
@pytest.mark.parametrize('body', [VALID_BODY, OVERSIZED_BODY])
def test_content_length_of_more_than_4300_digits_returns_400_body_too_large(digits, body):
    client, nav, _ = _nav_client()
    response = _post_with_content_length(client, '9' * digits, body)
    assert response.status_code == 400
    assert response.json()['error'] == 'body_too_large'
    assert nav.goals == []


@pytest.mark.parametrize('declared', ['1025', '99999999', '4300' + '0' * 10])
def test_declared_content_length_over_the_limit_returns_400_body_too_large(declared):
    client, nav, _ = _nav_client()
    response = _post_with_content_length(client, declared, VALID_BODY)
    assert response.status_code == 400
    assert response.json()['error'] == 'body_too_large'
    assert nav.goals == []


def test_the_server_keeps_working_after_hostile_content_length_requests():
    client, nav, _ = _nav_client()
    _post_with_content_length(client, b'\xb2', OVERSIZED_BODY)
    _post_with_content_length(client, '9' * 5000, VALID_BODY)
    assert _post(client, '/api/goal', GOOD).status_code == 200


class GateNav(FakeTeleopNav):
    """FakeTeleopNav whose teleop() is decided by the REAL TeleopGate (clock fixed)."""

    def __init__(self):
        super().__init__()
        from mtrb_bringup.web_utils import TeleopGate
        self.gate = TeleopGate(0.2, 0.8, 0.3, lambda: 0.0)

    def teleop(self, client_id, message):
        self.calls.append(('teleop', client_id, message))
        return self.gate.on_teleop(client_id, message, False).error


@pytest.mark.parametrize('field', ['vx', 'wz', 'seq'])
def test_ws_teleop_with_a_huge_integer_gets_an_error_and_the_server_stays_up(field):
    nav = GateNav()
    client = TestClient(create_app(StateStore(), TOKEN, nav_port=nav))
    message = dict(TELEOP, **{field: HUGE})
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(json.dumps(message))
        reply = json.loads(ws.receive_text())
        assert reply['code'] in ('invalid_teleop', 'unsupported_message')
        _sync(ws)                                    # the socket and the loop are alive
    with client, client.websocket_connect(WS_URL) as ws:    # and the server accepts others
        ws.send_text(json.dumps(dict(TELEOP, seq=0)))
        _sync(ws)
    assert nav.of('teleop')[-1][2]['seq'] == 0


def test_ws_teleop_with_a_huge_integer_never_publishes_anything_through_the_gate():
    nav = GateNav()
    client = TestClient(create_app(StateStore(), TOKEN, nav_port=nav))
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(json.dumps(dict(TELEOP, vx=HUGE)))
        assert json.loads(ws.receive_text())['code'] == 'invalid_teleop'
    assert nav.gate.owner_age_s is None              # nobody took the lease


def test_ws_message_with_an_integer_over_4300_digits_is_unsupported_not_a_crash():
    client, nav = _teleop_app(max_ws_bytes=16384)
    text = '{"type":"teleop","vx":1' + '0' * 5000 + ',"wz":0,"seq":1}'
    with client, client.websocket_connect(WS_URL) as ws:
        ws.send_text(text)
        reply = json.loads(ws.receive_text())
        assert reply['code'] == 'unsupported_message'
        assert nav.of('teleop') == []                # never reached the node
        _sync(ws)


# --- security headers on every HTTP response (phase 5 hardening) ---------------------------

EXPECTED_CSP = ("default-src 'self'; script-src 'self'; style-src 'self'; "
                "connect-src 'self' ws: wss:; img-src 'self' data: blob:; "
                "frame-ancestors 'none'; base-uri 'none'; form-action 'self'")
EXPECTED_SECURITY_HEADERS = {
    'x-frame-options': 'DENY',
    'x-content-type-options': 'nosniff',
    'referrer-policy': 'no-referrer',
    'content-security-policy': EXPECTED_CSP,
}
INTERNAL_SECRET = '/home/pi/secret_dir/internal.py LEAK-42'


class _FileMapPort:
    """MapPort double that writes a tiny map, so a first save succeeds."""

    def ready(self):
        return True

    def save_map(self, base_path, timeout_s):
        for ext in ('.yaml', '.pgm'):
            with open(base_path + ext, 'wb') as handle:
                handle.write(b'x')
        return True

    def serialize_posegraph(self, base_path, timeout_s):
        return False


class _ExplodingStore(StateStore):
    """StateStore whose snapshot raises an unexpected error (provokes the generic 500)."""

    def snapshot(self):
        raise RuntimeError(INTERNAL_SECRET)


def _assert_security_headers(response):
    for name, value in EXPECTED_SECURITY_HEADERS.items():
        assert response.headers.get(name) == value, (response.status_code, name)


def test_the_csp_constant_is_exactly_the_documented_policy():
    from mtrb_bringup.web_server import SECURITY_HEADERS
    assert {k.lower(): v for k, v in SECURITY_HEADERS.items()} == EXPECTED_SECURITY_HEADERS


def _map_client(tmp_path, clock, mode='mapping', min_free_mb=0):
    import os
    from unittest import mock
    from mtrb_bringup import web_server
    store = StateStore()
    store.update(mode=mode)
    real_saver = web_server.MapSaver

    def saver(*args, **kwargs):
        return real_saver(*args, clock=clock, **kwargs)

    with mock.patch.object(web_server, 'MapSaver', saver):
        app = create_app(store, TOKEN, map_port=_FileMapPort(),
                         map_dir=os.path.join(os.path.realpath(str(tmp_path)), 'maps'),
                         map_min_free_mb=min_free_mb, map_min_interval_s=5.0)
    return TestClient(app)


def test_security_headers_on_a_200(client):
    response = client.get('/api/state', headers=AUTH)
    assert response.status_code == 200
    _assert_security_headers(response)


def test_security_headers_on_the_served_static_files(tmp_path):
    for name in ('index.html', 'app.js', 'style.css'):
        (tmp_path / name).write_text('x')
    client = TestClient(create_app(StateStore(), TOKEN, str(tmp_path)))
    for path in ('/', '/app.js', '/style.css'):
        response = client.get(path)
        assert response.status_code == 200
        _assert_security_headers(response)


def test_security_headers_on_a_401(client):
    response = client.get('/api/state')
    assert response.status_code == 401
    _assert_security_headers(response)


def test_security_headers_on_a_404_of_an_unknown_route(client):
    response = client.get('/no/such/route', headers=AUTH)
    assert response.status_code == 404
    _assert_security_headers(response)


def test_security_headers_on_the_404_of_a_missing_web_ui(client):
    response = client.get('/')
    assert response.status_code == 404
    _assert_security_headers(response)


def test_security_headers_on_a_405(client):
    response = client.get('/api/goal', headers=AUTH)
    assert response.status_code == 405
    _assert_security_headers(response)


def test_security_headers_on_a_400(client):
    response = client.post('/api/goal', json={'x': 'bad'}, headers=AUTH)
    assert response.status_code == 400
    _assert_security_headers(response)


def test_security_headers_on_a_409(tmp_path):
    client = _map_client(tmp_path, lambda: 0.0, mode='map')
    response = client.post('/api/map/save', json={'name': 'lab'}, headers=AUTH)
    assert (response.status_code, response.json()['error']) == (409, 'wrong_mode')
    _assert_security_headers(response)


def test_security_headers_on_a_429(tmp_path):
    clock = [100.0]
    client = _map_client(tmp_path, lambda: clock[0])
    assert client.post('/api/map/save', json={'name': 'a'}, headers=AUTH).status_code == 200
    response = client.post('/api/map/save', json={'name': 'b'}, headers=AUTH)
    assert (response.status_code, response.json()['error']) == (429, 'save_rate_limited')
    _assert_security_headers(response)


def test_security_headers_on_a_503(client):
    response = client.post('/api/stop', headers=AUTH)
    assert response.status_code == 503
    _assert_security_headers(response)


def test_security_headers_on_a_507(tmp_path, monkeypatch):
    from collections import namedtuple
    from mtrb_bringup import web_server
    usage = namedtuple('usage', 'total used free')
    monkeypatch.setattr(web_server.shutil, 'disk_usage', lambda path: usage(1, 1, 0))
    client = _map_client(tmp_path, lambda: 0.0, min_free_mb=1)
    response = client.post('/api/map/save', json={'name': 'a'}, headers=AUTH)
    assert (response.status_code, response.json()['error']) == (507, 'insufficient_storage')
    _assert_security_headers(response)


def test_security_headers_on_the_generic_500_without_leaking_anything():
    client = TestClient(create_app(_ExplodingStore(), TOKEN), raise_server_exceptions=False)
    response = client.get('/api/state', headers=AUTH)
    assert response.status_code == 500
    assert response.json() == {'error': 'internal_error', 'detail': 'Internal error'}
    _assert_security_headers(response)
    everything = response.text + str(dict(response.headers))
    for forbidden in ('LEAK-42', '/home/pi', 'secret_dir', 'internal.py', 'Traceback',
                      'RuntimeError', 'web_server', '.py'):
        assert forbidden not in everything, forbidden


def test_the_generic_500_is_json_and_the_server_keeps_answering_afterwards():
    client = TestClient(create_app(_ExplodingStore(), TOKEN), raise_server_exceptions=False)
    assert client.get('/api/state', headers=AUTH).status_code == 500
    assert client.get('/api/state', headers=AUTH).status_code == 500
    assert client.get('/api/state').status_code == 401        # auth still checked first
    assert client.post('/api/stop', headers=AUTH).status_code == 503


def test_security_headers_are_not_overridable_by_the_request(client):
    response = client.get('/api/state', headers={**AUTH, 'X-Frame-Options': 'ALLOWALL'})
    assert response.headers['x-frame-options'] == 'DENY'


# --- is_placeholder_token: case-insensitive after strip -------------------------------------

@pytest.mark.parametrize('token', ['change-me-x', '  Change-Me', 'CHANGE-ME',
                                   'cHaNgE-mE-xxxxxxxxxxxxxxxx', 'change-me', '\tchange-me-1\n'])
def test_placeholder_tokens_are_detected_whatever_the_case_and_the_outer_spaces(token):
    assert is_placeholder_token(token) is True


@pytest.mark.parametrize('token', ['changeme', 'xCHANGE-ME', '', '   ', 'CHANGE_ME',
                                   'CHANGE ME', 'a-change-me-token'])
def test_other_tokens_are_not_placeholders(token):
    assert is_placeholder_token(token) is False


# === Audit (task 21): token loading, brute-force limiter, Origin, stop flood ============
# SECURITY: these tests fix the SAFE behaviour. No sleeping: every clock is injected.

SENTINEL = 'SENTINEL-9f3c2a7b-do-not-leak-0042'   # unique secret: must never appear anywhere


def _write(path, content, mode=0o600):
    data = content.encode() if isinstance(content, str) else content
    fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    try:
        os.write(fd, data)
    finally:
        os.close(fd)
    os.chmod(str(path), mode)
    return str(path)


def _everything_printable(err):
    """Return what logs and tracebacks show: str/repr/args of `err` and the formatted chain."""
    return ' '.join([str(err), repr(err), repr(err.args), ''.join(
        traceback.format_exception(type(err), err, err.__traceback__))])


def _assert_no_cause_chain_leak(err, with_context=False):
    """`raise ... from None`: no explicit cause, context hidden, nothing leaks the SENTINEL."""
    assert err.__cause__ is None
    assert err.__suppress_context__ is True
    assert SENTINEL not in _everything_printable(err)
    if with_context:
        link = err.__context__
        assert link is None or SENTINEL not in (
            repr(link) + repr(link.args) + str(link))


# --- read_token_file ------------------------------------------------------------------

def test_token_file_absent_returns_none(tmp_path):
    assert read_token_file(str(tmp_path / 'missing')) is None


@pytest.mark.parametrize('mode', [0o600, 0o400])
def test_token_file_private_modes_are_accepted(tmp_path, mode):
    path = _write(tmp_path / 'tok', 'a-valid-token-value', mode)
    assert read_token_file(path) == 'a-valid-token-value'


@pytest.mark.parametrize('mode', [0o644, 0o640, 0o604, 0o660, 0o606, 0o620, 0o602, 0o777])
def test_token_file_with_group_or_other_access_is_refused(tmp_path, mode):
    path = _write(tmp_path / 'tok', SENTINEL, mode)
    with pytest.raises(TokenError) as info:
        read_token_file(path)
    assert 'chmod 600' in str(info.value)
    assert SENTINEL not in _everything_printable(info.value)


def test_token_file_trailing_newlines_and_spaces_are_stripped(tmp_path):
    path = _write(tmp_path / 'tok', '  tok-with-newline-xyz \r\n\n')
    assert read_token_file(path) == 'tok-with-newline-xyz'


def test_token_file_empty_returns_empty_string(tmp_path):
    assert read_token_file(_write(tmp_path / 'tok', '')) == ''
    assert read_token_file(_write(tmp_path / 'tok2', ' \n\t\n')) == ''


def test_token_file_of_exactly_the_maximum_size_is_accepted(tmp_path):
    assert read_token_file(_write(tmp_path / 'tok', 'a' * TOKEN_FILE_MAX_BYTES)) == (
        'a' * TOKEN_FILE_MAX_BYTES)


def test_token_file_larger_than_1024_bytes_is_refused_without_leaking(tmp_path):
    path = _write(tmp_path / 'tok', SENTINEL + 'a' * TOKEN_FILE_MAX_BYTES)
    with pytest.raises(TokenError) as info:
        read_token_file(path)
    assert 'too large' in str(info.value)
    assert SENTINEL not in _everything_printable(info.value)


def test_token_file_with_invalid_utf8_is_refused_without_leaking(tmp_path):
    path = _write(tmp_path / 'tok', SENTINEL.encode() + b'\xff\xfe\x80')
    with pytest.raises(TokenError) as info:
        read_token_file(path)
    assert 'UTF-8' in str(info.value)
    assert SENTINEL not in _everything_printable(info.value)


def test_token_file_invalid_utf8_cause_chain_does_not_carry_the_content(tmp_path):
    path = _write(tmp_path / 'tok', SENTINEL.encode() + b'\xff\xfe\x80')
    with pytest.raises(TokenError) as info:
        read_token_file(path)
    _assert_no_cause_chain_leak(info.value)


def test_token_file_symlink_is_refused_even_to_a_private_file(tmp_path):
    target = _write(tmp_path / 'real', SENTINEL)
    link = tmp_path / 'link'
    os.symlink(target, link)
    with pytest.raises(TokenError) as info:
        read_token_file(str(link))
    assert 'regular file' in str(info.value)
    assert SENTINEL not in _everything_printable(info.value)


def test_token_file_dangling_symlink_is_refused_not_treated_as_absent(tmp_path):
    link = tmp_path / 'link'
    os.symlink(tmp_path / 'nowhere', link)
    with pytest.raises(TokenError):
        read_token_file(str(link))


def test_token_file_directory_is_refused(tmp_path):
    directory = tmp_path / 'dir'
    directory.mkdir(mode=0o700)
    with pytest.raises(TokenError) as info:
        read_token_file(str(directory))
    assert 'regular file' in str(info.value)


def test_token_file_fifo_is_refused_without_blocking(tmp_path):
    fifo = tmp_path / 'fifo'
    os.mkfifo(fifo, 0o600)
    with pytest.raises(TokenError) as info:
        read_token_file(str(fifo))
    assert 'regular file' in str(info.value)


def test_token_file_owned_by_another_user_is_refused(tmp_path, monkeypatch):
    path = _write(tmp_path / 'tok', SENTINEL)
    real_euid = os.geteuid()
    with monkeypatch.context() as scoped:
        scoped.setattr(os, 'geteuid', lambda: real_euid + 1)
        with pytest.raises(TokenError) as info:
            read_token_file(path)
    assert 'owned by the user' in str(info.value)
    assert SENTINEL not in _everything_printable(info.value)


def test_token_file_owner_is_taken_from_the_opened_descriptor(tmp_path, monkeypatch):
    path = _write(tmp_path / 'tok', SENTINEL)
    real = os.fstat

    def foreign(fd):
        info = real(fd)
        return types.SimpleNamespace(st_mode=info.st_mode, st_uid=info.st_uid + 1)

    with monkeypatch.context() as scoped:
        scoped.setattr(os, 'fstat', foreign)
        with pytest.raises(TokenError, match='owned by the user'):
            read_token_file(path)


def test_token_file_read_error_message_has_no_content(tmp_path, monkeypatch):
    path = _write(tmp_path / 'tok', SENTINEL)

    def boom(fd, size):
        raise OSError(5, 'I/O error')

    with monkeypatch.context() as scoped:
        scoped.setattr(os, 'read', boom)
        with pytest.raises(TokenError) as info:
            read_token_file(path)
    assert SENTINEL not in _everything_printable(info.value)


@pytest.mark.parametrize('path', ['relative/token', 'token', './token', '', None, 42,
                                  '~nonexistent_user_zzz9/token'])
def test_token_file_path_must_be_absolute(path):
    with pytest.raises(TokenError, match='must be an absolute path'):
        read_token_file(path)


def test_token_file_tilde_is_expanded_to_the_home_directory(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    _write(tmp_path / 'tok', 'tilde-expanded-token')
    assert read_token_file('~/tok') == 'tilde-expanded-token'


def test_token_file_opened_with_nofollow_flag(tmp_path, monkeypatch):
    path = _write(tmp_path / 'tok', 'a-valid-token-value')
    flags = []
    real = os.open

    def spy(p, f, *args, **kwargs):
        flags.append(f)
        return real(p, f, *args, **kwargs)

    with monkeypatch.context() as scoped:
        scoped.setattr(os, 'open', spy)
        read_token_file(path)
    assert flags and all(f & os.O_NOFOLLOW for f in flags)


# --- load_token -----------------------------------------------------------------------

def test_load_token_environment_has_priority_over_the_file(tmp_path):
    path = _write(tmp_path / 'tok', 'from-the-file-xxxxxxxxxxxx')
    assert load_token(path, {'MTRB_WEB_TOKEN': 'from-env-xxxxxxxxxxxxxxxx'}) == (
        'from-env-xxxxxxxxxxxxxxxx')


def test_load_token_environment_value_is_stripped():
    assert load_token('/nonexistent/x', {'MTRB_WEB_TOKEN': '  \tenv-token-padded \n'}) == (
        'env-token-padded')


def test_load_token_environment_present_never_touches_an_unsafe_file(tmp_path):
    path = _write(tmp_path / 'tok', SENTINEL, 0o644)   # would raise if it were read
    assert load_token(path, {'MTRB_WEB_TOKEN': 'env-token-wins'}) == 'env-token-wins'


@pytest.mark.parametrize('value', ['', '   ', '\n\t'])
def test_load_token_blank_environment_falls_back_to_the_file(tmp_path, value):
    path = _write(tmp_path / 'tok', 'from-the-file\n')
    assert load_token(path, {'MTRB_WEB_TOKEN': value}) == 'from-the-file'


def test_load_token_uses_the_process_environment_by_default(tmp_path, monkeypatch):
    monkeypatch.setenv('MTRB_WEB_TOKEN', 'process-env-token')
    assert load_token(str(tmp_path / 'missing')) == 'process-env-token'


def test_load_token_without_any_source_returns_empty_string(tmp_path):
    assert load_token(str(tmp_path / 'missing'), {}) == ''


def test_load_token_empty_file_returns_empty_string(tmp_path):
    assert load_token(_write(tmp_path / 'tok', '\n'), {}) == ''


def test_load_token_unsafe_file_raises_token_error_without_leaking(tmp_path):
    path = _write(tmp_path / 'tok', SENTINEL, 0o666)
    with pytest.raises(TokenError) as info:
        load_token(path, {})
    assert SENTINEL not in _everything_printable(info.value)


def test_load_token_bad_path_raises_token_error():
    with pytest.raises(TokenError, match='absolute'):
        load_token('relative/token', {})


def test_token_constants_are_the_audited_ones():
    assert TOKEN_MIN_LENGTH == 20
    assert TOKEN_FILE_MAX_BYTES == 1024
    assert TOKEN_ENV_VAR == 'MTRB_WEB_TOKEN'
    assert DEFAULT_TOKEN_FILE == '~/.config/mtrb/web_token'
    assert (AUTH_MAX_FAILURES, AUTH_WINDOW_S, AUTH_BLOCK_S) == (10, 60.0, 60.0)
    assert AUTH_MAX_TRACKED_IPS == 1024


# --- is_weak_token ----------------------------------------------------------------------

@pytest.mark.parametrize('token', [
    'CHANGE-ME', 'change-me-' + 'x' * 30, 'Change-Me-please-0123456789', '  cHaNgE-mE',
    'a' * 19, 'abcdefghijklmnopqrs', '', '   ', '\n\t ', 'a' * 20, 'a' * 64, ' ' * 20,
    '0' * 40, 'ab' * 5, '  ' + 'z' * 18 + '  '])
def test_weak_tokens_are_detected(token):
    assert is_weak_token(token) is True


@pytest.mark.parametrize('token', [
    'abcdefghijklmnopqrst', 'a' * 19 + 'b', 'aaaaaaaaaaaaaaaaaaab', 'S3cr3t-Value-0123456789',
    '  abcdefghijklmnopqrst  ', 'x1' * 10, 'a-random-token-of-24-chars'])
def test_varied_tokens_of_at_least_20_characters_are_strong(token):
    assert is_weak_token(token) is False


def test_a_token_of_19_varied_characters_is_weak_and_of_20_is_strong():
    assert is_weak_token('abcdefghijklmnopqrs') is True      # 19
    assert is_weak_token('abcdefghijklmnopqrst') is False    # 20


# --- AuthFailureLimiter -----------------------------------------------------------------

class Clock:
    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now


def _fail(limiter, ip, times):
    return [limiter.record_failure(ip) for _ in range(times)]


def test_limiter_blocks_exactly_at_the_tenth_failure():
    limiter = AuthFailureLimiter(clock=Clock())
    results = _fail(limiter, '10.0.0.1', 9)
    assert results == [False] * 9
    assert limiter.blocked('10.0.0.1') is False
    assert limiter.record_failure('10.0.0.1') is True       # failure no. 10
    assert limiter.blocked('10.0.0.1') is True
    assert limiter.record_failure('10.0.0.1') is True       # failure no. 11
    assert limiter.blocked('10.0.0.1') is True


def test_limiter_block_lasts_60_seconds_and_is_not_extended_by_more_failures():
    clock = Clock()
    limiter = AuthFailureLimiter(clock=clock)
    _fail(limiter, 'a', 10)
    clock.now += 59.0
    _fail(limiter, 'a', 5)                  # attempts while blocked do not extend the block
    assert limiter.blocked('a') is True
    clock.now += 1.0                        # 60 s after the block started
    assert limiter.blocked('a') is False
    assert limiter.record_failure('a') is False   # fresh count after the block


def test_limiter_failures_expire_after_the_window():
    clock = Clock()
    limiter = AuthFailureLimiter(clock=clock)
    _fail(limiter, 'a', 9)
    clock.now += 60.0                       # window over: the 9 failures are forgotten
    assert _fail(limiter, 'a', 9) == [False] * 9
    assert limiter.blocked('a') is False
    assert limiter.record_failure('a') is True


def test_limiter_failures_inside_the_window_still_count():
    clock = Clock()
    limiter = AuthFailureLimiter(clock=clock)
    _fail(limiter, 'a', 9)
    clock.now += 59.9
    assert limiter.record_failure('a') is True


def test_limiter_window_restarts_after_expiry():
    clock = Clock()
    limiter = AuthFailureLimiter(clock=clock)
    _fail(limiter, 'a', 5)
    clock.now += 61.0
    assert _fail(limiter, 'a', 4) == [False] * 4     # new window: count 4
    clock.now += 30.0                                # 30 s into the NEW window
    assert limiter.record_failure('a') is False      # 5th of the new window
    clock.now += 31.0                                # 61 s since the new window began
    assert _fail(limiter, 'a', 9) == [False] * 9     # forgotten again
    assert limiter.record_failure('a') is True


def test_limiter_blocked_ip_does_not_affect_another_ip():
    limiter = AuthFailureLimiter(clock=Clock())
    _fail(limiter, 'bad', 10)
    assert limiter.blocked('bad') is True
    assert limiter.blocked('good') is False
    assert limiter.record_failure('good') is False
    assert limiter.blocked('good') is False


def test_limiter_unknown_ip_is_not_blocked_and_not_tracked():
    limiter = AuthFailureLimiter(clock=Clock())
    assert limiter.blocked('never-seen') is False
    assert len(limiter) == 0


def test_limiter_clock_going_back_never_keeps_an_ip_blocked_for_longer():
    clock = Clock(1000.0)
    limiter = AuthFailureLimiter(clock=clock)
    _fail(limiter, 'a', 10)
    clock.now = 400.0                       # the clock jumped back by 10 minutes
    assert limiter.blocked('a') is False    # entry discarded, not blocked "for 600 s"
    _fail(limiter, 'b', 9)
    clock.now = 100.0
    assert limiter.record_failure('b') is False     # stale entry dropped: count restarts


def test_limiter_table_is_bounded_to_the_default_1024_ips():
    limiter = AuthFailureLimiter(clock=Clock())
    for index in range(AUTH_MAX_TRACKED_IPS * 2 + 7):
        limiter.record_failure(f'10.{index // 256}.{index % 256}.1')
        assert len(limiter) <= AUTH_MAX_TRACKED_IPS
    assert len(limiter) == AUTH_MAX_TRACKED_IPS


def test_limiter_full_table_drops_expired_entries_before_blocked_ones():
    clock = Clock(0.0)
    limiter = AuthFailureLimiter(clock=clock, max_ips=2)
    _fail(limiter, 'attacker', 9)
    clock.now = 10.0
    limiter.record_failure('victim-of-expiry')      # created later, expires first
    clock.now = 50.0
    assert limiter.record_failure('attacker') is True    # blocked until t=110
    clock.now = 75.0                                # 'victim-of-expiry' (t=10) is stale
    limiter.record_failure('newcomer')              # table full -> make room
    assert len(limiter) == 2
    assert limiter.blocked('attacker') is True      # the block survived the eviction


def test_limiter_full_table_evicts_the_oldest_when_nothing_expired():
    clock = Clock(0.0)
    limiter = AuthFailureLimiter(clock=clock, max_ips=3)
    for ip in ('oldest', 'middle', 'newest'):
        _fail(limiter, ip, 9)
        clock.now += 1.0
    limiter.record_failure('newcomer')              # nothing expired: 'oldest' is dropped
    assert len(limiter) == 3
    assert limiter.record_failure('oldest') is False      # its 9 failures were forgotten
    assert limiter.record_failure('newest') is True       # still tracked: 10th failure


def test_limiter_valid_requests_never_create_entries():
    limiter = AuthFailureLimiter(clock=Clock())
    app = create_app(StateStore(), TOKEN, auth_limiter=limiter)
    client = TestClient(app)
    for _ in range(40):
        assert client.get('/api/state', headers=AUTH).status_code == 200
    assert len(limiter) == 0


# --- brute force over REST --------------------------------------------------------------

BAD = {'Authorization': f'Bearer wrong-{SENTINEL}'}


def _limited_app(limiter=None, nav=None, on_event=None, **kwargs):
    limiter = AuthFailureLimiter(clock=Clock()) if limiter is None else limiter
    app = create_app(StateStore(), TOKEN, auth_limiter=limiter, nav_port=nav,
                     on_event=on_event, **kwargs)
    return app, limiter


def test_failure_number_10_gets_401_and_number_11_gets_429_with_retry_after():
    app, _ = _limited_app()
    client = TestClient(app)
    statuses = [client.get('/api/state', headers=BAD).status_code for _ in range(12)]
    assert statuses == [401] * 10 + [429] * 2
    response = client.get('/api/state', headers=BAD)
    assert response.status_code == 429
    assert response.headers['retry-after'] == '60'
    assert response.json()['error'] == 'too_many_attempts'
    assert 'www-authenticate' not in response.headers


def test_valid_token_from_a_blocked_ip_gets_429_on_every_rest_endpoint():
    app, limiter = _limited_app(nav=FakeNav())
    client = TestClient(app)
    for _ in range(10):
        client.get('/api/state', headers=BAD)
    assert limiter.blocked('testclient')
    for method, path in (('get', '/api/state'), ('post', '/api/goal'), ('post', '/api/cancel'),
                         ('post', '/api/dock'), ('post', '/api/initialpose'),
                         ('post', '/api/map/save'), ('get', '/api/maps')):
        response = getattr(client, method)(path, headers=AUTH)
        assert response.status_code == 429, path
        assert response.headers['retry-after'] == '60'


def test_blocked_ip_is_released_after_60_seconds():
    clock = Clock()
    app, _ = _limited_app(AuthFailureLimiter(clock=clock))
    client = TestClient(app)
    for _ in range(10):
        client.get('/api/state', headers=BAD)
    assert client.get('/api/state', headers=AUTH).status_code == 429
    clock.now += 59.0
    assert client.get('/api/state', headers=AUTH).status_code == 429
    clock.now += 1.0
    assert client.get('/api/state', headers=AUTH).status_code == 200


def test_another_ip_is_not_affected_by_a_blocked_one():
    app, _ = _limited_app()
    attacker = TestClient(app, client=('203.0.113.9', 4000))
    friend = TestClient(app, client=('192.168.1.20', 4001))
    for _ in range(11):
        attacker.get('/api/state', headers=BAD)
    assert attacker.get('/api/state', headers=AUTH).status_code == 429
    assert friend.get('/api/state', headers=AUTH).status_code == 200
    assert friend.get('/api/state', headers=BAD).status_code == 401


def test_x_forwarded_for_is_ignored_the_socket_peer_is_what_counts():
    app, limiter = _limited_app()
    client = TestClient(app)
    for index in range(10):         # the attacker rotates fake forwarded addresses
        response = client.get('/api/state', headers={**BAD, 'X-Forwarded-For': f'9.9.9.{index}',
                                                     'X-Real-IP': f'8.8.8.{index}'})
        assert response.status_code == 401
    assert limiter.blocked('testclient') is True
    assert len(limiter) == 1
    response = client.get('/api/state', headers={**AUTH, 'X-Forwarded-For': '1.2.3.4'})
    assert response.status_code == 429


def test_a_forwarded_header_cannot_borrow_the_identity_of_a_clean_ip():
    app, limiter = _limited_app()
    client = TestClient(app, client=('203.0.113.9', 4000))
    for _ in range(10):
        client.get('/api/state', headers=BAD)
    response = client.get('/api/state', headers={**AUTH, 'X-Forwarded-For': '192.168.1.20'})
    assert response.status_code == 429


def test_only_failures_feed_the_limiter_valid_requests_in_between_do_not_reset_it():
    app, limiter = _limited_app()
    client = TestClient(app)
    for _ in range(9):
        client.get('/api/state', headers=BAD)
        assert client.get('/api/state', headers=AUTH).status_code == 200
    assert limiter.blocked('testclient') is False
    assert client.get('/api/state', headers=BAD).status_code == 401    # the 10th failure
    assert limiter.blocked('testclient') is True


def test_missing_authorization_header_counts_as_a_failure():
    app, _ = _limited_app()
    client = TestClient(app)
    assert [client.get('/api/state').status_code for _ in range(11)] == [401] * 10 + [429]


def test_invalid_token_from_a_blocked_ip_is_429_and_does_not_extend_the_block():
    clock = Clock()
    app, _ = _limited_app(AuthFailureLimiter(clock=clock))
    client = TestClient(app)
    for _ in range(10):
        client.get('/api/state', headers=BAD)
    clock.now += 30.0
    assert client.get('/api/state', headers=BAD).status_code == 429
    clock.now += 30.0                        # 60 s after the block began
    assert client.get('/api/state', headers=AUTH).status_code == 200


def test_the_default_limiter_is_created_when_none_is_given():
    client = TestClient(create_app(StateStore(), TOKEN))
    statuses = [client.get('/api/state', headers=BAD).status_code for _ in range(11)]
    assert statuses == [401] * 10 + [429]


def test_auth_logs_are_summarised_and_never_contain_ip_token_or_authorization():
    events = []
    app, _ = _limited_app(on_event=lambda level, text: events.append((level, text)))
    client = TestClient(app, client=('203.0.113.77', 4444))
    for _ in range(25):
        client.get('/api/state', headers=BAD)
    client.get('/api/state', headers=AUTH)
    assert 1 <= len(events) <= 3         # a flood is one line per period, not one per attempt
    for level, text in events:
        assert level == 'warning'
        for secret in ('203.0.113.77', SENTINEL, TOKEN, 'Bearer', 'Authorization',
                       'testclient', 'wrong'):
            assert secret not in text
    assert any('failed authentication' in text for _, text in events)
    assert any('too many failed tokens' in text for _, text in events)


# --- the stop exception ---------------------------------------------------------------------

def _blocked_stop_app(nav=None):
    nav = FakeTeleopNav() if nav is None else nav
    app, limiter = _limited_app(nav=nav)
    client = TestClient(app)
    for _ in range(10):
        client.get('/api/state', headers=BAD)
    assert limiter.blocked('testclient')
    return client, nav, limiter


def test_stop_with_a_valid_token_always_passes_even_from_a_blocked_ip():
    client, nav, _ = _blocked_stop_app()
    for _ in range(3):
        response = client.post('/api/stop', headers=AUTH)
        assert response.status_code == 200
        assert response.json() == {'ok': True}
    assert nav.calls == [('stop',)] * 3


def test_stop_with_an_invalid_token_from_a_blocked_ip_is_429_and_does_not_stop():
    client, nav, _ = _blocked_stop_app()
    response = client.post('/api/stop', headers=BAD)
    assert response.status_code == 429
    assert response.headers['retry-after'] == '60'
    assert nav.calls == []


def test_stop_without_a_token_from_a_blocked_ip_is_429_and_does_not_stop():
    client, nav, _ = _blocked_stop_app()
    assert client.post('/api/stop').status_code == 429
    assert nav.calls == []


def test_stop_with_an_invalid_token_from_a_clean_ip_is_401_and_feeds_the_limiter():
    nav = FakeTeleopNav()
    app, limiter = _limited_app(nav=nav)
    client = TestClient(app)
    assert [client.post('/api/stop', headers=BAD).status_code for _ in range(11)] == (
        [401] * 10 + [429])
    assert nav.calls == []
    assert limiter.blocked('testclient') is True


def test_valid_stops_never_feed_the_limiter():
    nav = FakeTeleopNav()
    app, limiter = _limited_app(nav=nav)
    client = TestClient(app)
    for _ in range(50):
        assert client.post('/api/stop', headers=AUTH).status_code == 200
    assert len(limiter) == 0
    assert len(nav.calls) == 50


def test_the_stop_exception_does_not_open_any_other_endpoint_for_a_blocked_ip():
    client, nav, _ = _blocked_stop_app()
    assert client.post('/api/cancel', headers=AUTH).status_code == 429
    assert client.get('/api/state', headers=AUTH).status_code == 429
    assert client.post('/api/stop', headers=AUTH).status_code == 200


def test_the_stop_dependency_and_handler_run_on_the_event_loop_not_in_the_thread_pool():
    app, _ = _limited_app(nav=FakeTeleopNav())
    route = next(r for r in app.routes if getattr(r, 'path', '') == '/api/stop')
    assert inspect.iscoroutinefunction(route.endpoint)
    dependencies = [d.call for d in route.dependant.dependencies]
    assert dependencies and all(inspect.iscoroutinefunction(call) for call in dependencies)
    state = next(r for r in app.routes if getattr(r, 'path', '') == '/api/state')
    assert all(inspect.iscoroutinefunction(d.call) for d in state.dependant.dependencies)


def test_a_rest_stop_never_waits_for_the_thread_pool(monkeypatch):
    import anyio.to_thread
    calls = []
    real = anyio.to_thread.run_sync

    async def spy(func, *args, **kwargs):
        calls.append(getattr(func, '__name__', repr(func)))
        return await real(func, *args, **kwargs)

    monkeypatch.setattr(anyio.to_thread, 'run_sync', spy)
    app, _ = _limited_app(nav=FakeTeleopNav())
    client = TestClient(app)
    assert client.post('/api/stop', headers=AUTH).status_code == 200
    assert calls == []
    client.post('/api/cancel', headers=AUTH)     # control: a sync handler does use the pool
    assert calls, 'the detector would not notice a thread pool hop'


# --- WebSocket: Origin, token and lock are decided BEFORE accept -----------------------------

def _ws_scope(query='', headers=(), client=('10.1.1.1', 5000)):
    return {'type': 'websocket', 'path': '/ws', 'raw_path': b'/ws', 'root_path': '',
            'scheme': 'ws', 'query_string': query.encode(), 'subprotocols': [],
            'headers': [(k.lower().encode(), v.encode()) for k, v in headers],
            'client': client, 'server': ('robot', 8080), 'asgi': {'version': '3.0'}}


def _handshake(app, query='', headers=(), client=('10.1.1.1', 5000)):
    """Run one WebSocket handshake at ASGI level; return the messages the app sent."""
    sent = []
    incoming = [{'type': 'websocket.connect'}]

    async def receive():
        if incoming:
            return incoming.pop(0)
        return {'type': 'websocket.disconnect', 'code': 1000}

    async def send(message):
        sent.append(message)

    asyncio.run(app(_ws_scope(query, headers, client), receive, send))
    return sent


def _accepted(sent):
    return any(m['type'] == 'websocket.accept' for m in sent)


def _refused_before_accept(sent):
    return (bool(sent) and not _accepted(sent) and sent[0]['type'] == 'websocket.close'
            and sent[0].get('code') == 1008)


def _ws_app_limited(limiter=None, **kwargs):
    from mtrb_bringup.web_server import BroadcastHub
    limiter = AuthFailureLimiter(clock=Clock()) if limiter is None else limiter
    hub = BroadcastHub()
    return create_app(StateStore(), TOKEN, hub=hub, auth_limiter=limiter, **kwargs), hub, limiter


GOOD_Q = f'token={TOKEN}'


@pytest.mark.parametrize('query', ['', 'token=', 'token=wrong', f'token={TOKEN}x',
                                   f'token={TOKEN[:-1]}'])
def test_ws_bad_token_is_refused_before_accept_never_accept_then_1008(query):
    app, hub, _ = _ws_app_limited()
    sent = _handshake(app, query, [('host', 'robot:8080')])
    assert _refused_before_accept(sent), sent
    assert hub.client_count == 0


def test_ws_valid_token_is_accepted_positive_control():
    app, hub, _ = _ws_app_limited()
    sent = _handshake(app, GOOD_Q, [('host', 'robot:8080')])
    assert sent[0]['type'] == 'websocket.accept'


@pytest.mark.parametrize('origin,host', [
    ('http://robot:8080', 'robot:8080'),
    ('http://ROBOT:8080', 'robot:8080'),
    ('http://robot', 'robot'),
    ('http://robot:80', 'robot'),
    ('http://robot', 'robot:80'),
    ('https://robot', 'robot'),
    ('https://robot:443', 'robot'),
    ('https://robot', 'robot:443'),
    ('http://192.168.1.50:8080', '192.168.1.50:8080'),
    ('http://[::1]:8080', '[::1]:8080')])
def test_ws_origin_equal_to_host_is_accepted(origin, host):
    app, _, _ = _ws_app_limited()
    sent = _handshake(app, GOOD_Q, [('host', host), ('origin', origin)])
    assert sent[0]['type'] == 'websocket.accept', (origin, host)


def test_ws_without_origin_header_is_accepted_non_browser_client():
    app, _, _ = _ws_app_limited()
    assert _handshake(app, GOOD_Q, [('host', 'robot:8080')])[0]['type'] == 'websocket.accept'


@pytest.mark.parametrize('origin,host', [
    ('http://evil.example', 'robot:8080'),
    ('http://robot:9999', 'robot:8080'),
    ('http://robot', 'robot:8080'),               # port 80 vs 8080
    ('http://robot:8080', 'robot'),               # 8080 vs default 80
    ('https://robot', 'robot:80'),                # 443 vs 80
    ('http://robot:443', 'robot'),                # 443 vs 80
    ('http://robot.evil.example', 'robot:8080'),
    ('http://evil.example/robot:8080', 'robot:8080'),
    ('http://robot:8080@evil.example', 'robot:8080'),
    ('http://user@robot:8080', 'robot:8080'),
    ('null', 'robot:8080'),
    ('NULL', 'robot:8080'),
    ('', 'robot:8080'),
    ('robot:8080', 'robot:8080'),                 # no scheme
    ('//robot:8080', 'robot:8080'),
    ('file:///etc/passwd', 'robot:8080'),
    ('ftp://robot:8080', 'robot:8080'),
    ('ws://robot:8080', 'robot:8080'),
    ('chrome-extension://abcdef', 'robot:8080'),
    ('http://robot:notaport', 'robot:8080'),
    ('http://robot:8080', 'robot:notaport')])
def test_ws_foreign_or_malformed_origin_is_refused_before_accept(origin, host):
    app, hub, _ = _ws_app_limited()
    sent = _handshake(app, GOOD_Q, [('host', host), ('origin', origin)])
    assert _refused_before_accept(sent), (origin, host, sent)
    assert hub.client_count == 0


def test_ws_origin_without_host_header_is_refused_before_accept():
    app, _, _ = _ws_app_limited()
    sent = _handshake(app, GOOD_Q, [('origin', 'http://robot:8080')])
    assert _refused_before_accept(sent), sent


def test_ws_foreign_origin_with_a_bad_token_is_refused_before_accept():
    app, _, _ = _ws_app_limited()
    sent = _handshake(app, 'token=wrong', [('host', 'robot:8080'),
                                           ('origin', 'http://evil.example')])
    assert _refused_before_accept(sent)


def test_ws_foreign_origin_does_not_leak_through_a_valid_token_in_the_url():
    events = []
    app, _, _ = _ws_app_limited(on_event=lambda level, text: events.append(text))
    _handshake(app, GOOD_Q, [('host', 'robot:8080'), ('origin', 'http://evil.example')])
    assert events and all(TOKEN not in t and 'evil.example' not in t for t in events)


def test_ws_valid_token_from_a_blocked_ip_is_refused_before_accept():
    app, hub, limiter = _ws_app_limited()
    for _ in range(10):
        limiter.record_failure('10.1.1.1')
    sent = _handshake(app, GOOD_Q, [('host', 'robot:8080')])
    assert _refused_before_accept(sent), sent
    assert hub.client_count == 0


def test_ws_invalid_token_from_a_blocked_ip_is_refused_before_accept_and_not_counted():
    clock = Clock()
    app, _, limiter = _ws_app_limited(AuthFailureLimiter(clock=clock))
    for _ in range(10):
        limiter.record_failure('10.1.1.1')
    assert _refused_before_accept(_handshake(app, 'token=wrong', [('host', 'robot:8080')]))
    clock.now += 60.0       # the block was NOT extended by the refused attempt
    assert _handshake(app, GOOD_Q, [('host', 'robot:8080')])[0]['type'] == 'websocket.accept'


def test_ws_blocked_ip_does_not_block_another_ip():
    app, _, limiter = _ws_app_limited()
    for _ in range(10):
        limiter.record_failure('10.1.1.1')
    sent = _handshake(app, GOOD_Q, [('host', 'robot:8080')], client=('10.2.2.2', 6000))
    assert sent[0]['type'] == 'websocket.accept'


def test_ws_ten_bad_tokens_block_the_ip_and_the_eleventh_is_refused_with_a_good_one():
    app, _, limiter = _ws_app_limited()
    for _ in range(10):
        assert _refused_before_accept(
            _handshake(app, 'token=wrong', [('host', 'robot:8080')]))
    assert limiter.blocked('10.1.1.1') is True
    assert _refused_before_accept(_handshake(app, GOOD_Q, [('host', 'robot:8080')]))


def test_ws_and_rest_share_the_same_failure_budget():
    app, hub, limiter = _ws_app_limited()
    client = TestClient(app, client=('10.1.1.1', 5000))
    for _ in range(5):
        _handshake(app, 'token=wrong', [('host', 'robot:8080')])
        client.get('/api/state', headers=BAD)
    assert limiter.blocked('10.1.1.1') is True
    assert client.get('/api/state', headers=AUTH).status_code == 429


def test_ws_x_forwarded_for_is_ignored():
    app, _, limiter = _ws_app_limited()
    for index in range(10):
        _handshake(app, 'token=wrong', [('host', 'robot:8080'),
                                        ('x-forwarded-for', f'7.7.7.{index}')],
                   client=('10.1.1.1', 5000 + index))
    assert limiter.blocked('10.1.1.1') is True
    sent = _handshake(app, GOOD_Q, [('host', 'robot:8080'), ('x-forwarded-for', '10.9.9.9')])
    assert _refused_before_accept(sent)
    assert _handshake(app, GOOD_Q, [('host', 'robot:8080'), ('x-forwarded-for', '10.1.1.1')],
                      client=('10.9.9.9', 1))[0]['type'] == 'websocket.accept'


def test_ws_accepted_connections_never_feed_the_limiter():
    app, _, limiter = _ws_app_limited()
    for _ in range(30):
        assert _handshake(app, GOOD_Q, [('host', 'robot:8080')])[0]['type'] == (
            'websocket.accept')
    assert len(limiter) == 0


def test_ws_refusals_are_logged_in_summary_without_ip_token_or_origin():
    events = []
    app, _, _ = _ws_app_limited(on_event=lambda level, text: events.append((level, text)))
    for _ in range(15):
        _handshake(app, f'token=wrong-{SENTINEL}', [('host', 'robot:8080'),
                                                    ('origin', 'http://evil.example')],
                   client=('203.0.113.5', 1))
    assert 1 <= len(events) <= 3
    for level, text in events:
        assert level == 'warning'
        for secret in ('203.0.113.5', SENTINEL, 'evil.example', 'wrong'):
            assert secret not in text
    assert any('Origin' in text for _, text in events)


def test_ws_foreign_origin_is_refused_with_the_test_client_too():
    app, hub, _ = _ws_app_limited()
    client = TestClient(app)
    from starlette.websockets import WebSocketDisconnect
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect(f'/ws?{GOOD_Q}',
                                      headers={'origin': 'http://evil.example'}) as ws:
            ws.receive_text()
    assert hub.client_count == 0
    with client.websocket_connect(f'/ws?{GOOD_Q}', headers={'origin': 'http://testserver'}):
        pass    # same-origin works


# === Cause chain of the public errors (raise ... from None) ===============================
# CLIENT DATA: the exceptions chained to ApiError/TokenError (UnicodeDecodeError, JSONDecodeError,
# ValueError of the name or goal) keep the body or the value in args/repr. Each site has a
# factory returning the public exception; two parametrized tests exercise all of them.

def _route(app, path):
    return next(r for r in app.routes if getattr(r, 'path', None) == path
                and 'POST' in getattr(r, 'methods', ()))


class _FakeRequest:
    def __init__(self, raw):
        self.headers = {}
        self._raw = raw

    async def stream(self):
        yield self._raw


def _carrier(base):
    """Exception type whose args/repr carry the value but whose str() is a fixed text."""
    return type('Carrier', (base,), {'__str__': lambda self: 'fixed text'})


def _site_token_file(tmp_path, monkeypatch):
    path = _write(tmp_path / 'tok', SENTINEL.encode() + b'\xff\xfe\x80')
    with pytest.raises(TokenError) as info:
        read_token_file(path)
    return info.value


def _read_json_site(raw):
    def site(tmp_path, monkeypatch):
        endpoint = _route(create_app(StateStore(), TOKEN), '/api/goal').endpoint
        reader = inspect.signature(endpoint).parameters['body'].default.dependency
        with pytest.raises(Exception) as info:
            asyncio.run(reader(_FakeRequest(raw)))
        assert info.value.code == 'invalid_json'
        return info.value
    return site


def _map_name_site(error_type):
    def site(tmp_path, monkeypatch):
        from mtrb_bringup import web_server

        def echoing(name):
            raise _carrier(getattr(web_server, error_type, ValueError))(name)

        monkeypatch.setattr(web_server, 'valid_map_name', echoing)
        with pytest.raises(web_server.ApiError) as info:
            web_server._map_save_from_body({'name': SENTINEL})
        return info.value
    return site


def _map_name_real_site(name):
    def site(tmp_path, monkeypatch):
        from mtrb_bringup import web_server
        with pytest.raises(web_server.ApiError) as info:
            web_server._map_save_from_body({'name': name})
        return info.value
    return site


def _echoing_validate_goal(*_args):
    raise _carrier(ValueError)(SENTINEL)


def _goal_site(path, echo):
    def site(tmp_path, monkeypatch):
        from mtrb_bringup import web_server
        if echo:
            monkeypatch.setattr(web_server, 'validate_goal', _echoing_validate_goal)
        client, _, _ = _nav_client()
        endpoint = _route(client.app, path).endpoint
        x = 5.0 if echo else 9000.0          # 9000 m: valid body, outside the 10 m map
        with pytest.raises(web_server.ApiError) as info:
            endpoint(body={'x': x, 'y': 1.0, 'yaw': 0.0})
        return info.value
    return site


CAUSE_SITES = {
    'token-file-non-utf8': (_site_token_file, TokenError, None),
    'json-non-utf8': (_read_json_site(SENTINEL.encode() + b'\xff\xfe\x80'), None,
                      'invalid_json'),
    'json-truncated': (_read_json_site(('{"name": "' + SENTINEL).encode()), None,
                       'invalid_json'),
    'json-deeply-nested': (_read_json_site(('[' * 990 + SENTINEL).encode()), None,
                           'invalid_json'),
    'map-name-weird': (_map_name_real_site(SENTINEL + '/../x'), None, 'invalid_name'),
    'map-name-reserved': (_map_name_real_site('MAP'), None, 'reserved_name'),
    'map-name-echoing-validator': (_map_name_site('ValueError'), None, 'invalid_name'),
    'map-name-reserved-echoing': (_map_name_site('ReservedMapNameError'), None,
                                  'reserved_name'),
    'goal-invalid': (_goal_site('/api/goal', True), None, 'invalid_goal'),
    'goal-outside-map': (_goal_site('/api/goal', False), None, 'invalid_goal'),
    'pose-invalid': (_goal_site('/api/initialpose', True), None, 'invalid_pose'),
    'pose-outside-map': (_goal_site('/api/initialpose', False), None, 'invalid_pose'),
}


@pytest.mark.parametrize('site_id', list(CAUSE_SITES))
def test_public_error_has_no_explicit_cause_and_prints_no_client_data(
        site_id, tmp_path, monkeypatch):
    factory, _type, code = CAUSE_SITES[site_id]
    err = factory(tmp_path, monkeypatch)
    if code is not None:
        assert err.code == code
    _assert_no_cause_chain_leak(err)


CONTEXT_LEAKS = {'token-file-non-utf8', 'json-non-utf8', 'map-name-echoing-validator',
                 'map-name-reserved-echoing', 'goal-invalid', 'pose-invalid'}
_CONTEXT_BUG = pytest.mark.xfail(strict=True, reason=(
    'BUG (backend, low): "raise ... from None" inside the except block leaves the original '
    'exception in err.__context__ (hidden when printing, but reachable; its args/repr carry '
    'the body/name/token). Fix: raise outside the except block, or clear err.__context__.'))


@pytest.mark.parametrize('site_id', [
    pytest.param(site, marks=_CONTEXT_BUG) if site in CONTEXT_LEAKS else site
    for site in CAUSE_SITES])
def test_public_error_context_attribute_does_not_carry_the_client_data(
        site_id, tmp_path, monkeypatch):
    err = CAUSE_SITES[site_id][0](tmp_path, monkeypatch)
    _assert_no_cause_chain_leak(err, with_context=True)
