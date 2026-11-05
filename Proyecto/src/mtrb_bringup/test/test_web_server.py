"""Unit tests for mtrb_bringup.web_server (no ROS needed; needs fastapi and httpx)."""

import json
import threading

import pytest

pytest.importorskip('fastapi')
pytest.importorskip('httpx')

from fastapi.testclient import TestClient  # noqa: E402, I100
from mtrb_bringup.web_server import (  # noqa: E402, I100
    create_app, is_placeholder_token, resolve_token, StateStore)

TOKEN = 'secret-token-123'
KEYS = {'mode', 'pose', 'scan_age', 'map', 'plan_len', 'nav', 'battery_pct', 'stamp',
        'dock'}


def _yaml(token_line):
    return f"mtrb_web_bridge:\n  ros__parameters:\n    auth:\n      token: {token_line}\n"


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


# --- resolve_token ------------------------------------------------------------------

def test_resolve_token_env_has_priority(tmp_path):
    (tmp_path / 'web.local.yaml').write_text(_yaml('local'))
    (tmp_path / 'web.yaml').write_text(_yaml('default'))
    assert resolve_token(str(tmp_path), {'MTRB_WEB_TOKEN': 'fromenv'}) == 'fromenv'


def test_resolve_token_local_over_default(tmp_path):
    (tmp_path / 'web.local.yaml').write_text(_yaml('local'))
    (tmp_path / 'web.yaml').write_text(_yaml('default'))
    assert resolve_token(str(tmp_path), {}) == 'local'


def test_resolve_token_falls_back_to_default(tmp_path):
    (tmp_path / 'web.yaml').write_text(_yaml('default'))
    assert resolve_token(str(tmp_path), {}) == 'default'


def test_resolve_token_empty_env_is_ignored(tmp_path):
    (tmp_path / 'web.yaml').write_text(_yaml('default'))
    assert resolve_token(str(tmp_path), {'MTRB_WEB_TOKEN': ''}) == 'default'


def test_resolve_token_broken_local_falls_back_to_default(tmp_path):
    (tmp_path / 'web.local.yaml').write_text('a: [unclosed\n  : :')
    (tmp_path / 'web.yaml').write_text(_yaml('default'))
    assert resolve_token(str(tmp_path), {}) == 'default'


@pytest.mark.parametrize('content', [
    'other: 1\n', '', 'just a string', 'mtrb_web_bridge:\n  ros__parameters: {}\n'])
def test_resolve_token_yaml_without_key_returns_empty(tmp_path, content):
    (tmp_path / 'web.yaml').write_text(content)
    assert resolve_token(str(tmp_path), {}) == ''


@pytest.mark.parametrize('value', ['12345', '[a, b]', 'null', 'true'])
def test_resolve_token_non_string_token_is_ignored(tmp_path, value):
    (tmp_path / 'web.yaml').write_text(_yaml(value))
    assert resolve_token(str(tmp_path), {}) == ''


def test_resolve_token_no_files_returns_empty(tmp_path):
    assert resolve_token(str(tmp_path), {}) == ''


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
    response = _post(client, '/api/initialpose', {'x': 1, 'y': -2, 'yaw': 3})
    assert response.status_code == 200
    assert response.json() == {'ok': True}
    assert nav.poses == [(1.0, -2.0, 3.0)]
    assert all(type(v) is float for v in nav.poses[0])
    assert nav.goals == []


def test_initialpose_does_not_need_a_map():
    client, nav, _ = _nav_client(grid=None, mode='map')
    assert _post(client, '/api/initialpose', GOOD).status_code == 200
    assert nav.poses == [(3.5, 2.5, 1.0)]


def test_initialpose_may_lie_outside_the_map():
    client, nav, _ = _nav_client(mode='map')
    assert _post(client, '/api/initialpose', {'x': 500, 'y': 500, 'yaw': 0}).status_code == 200


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
