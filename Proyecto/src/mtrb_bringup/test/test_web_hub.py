"""Unit tests for ClientChannel and BroadcastHub (pure Python, no ROS; fastapi needed)."""

import pytest

pytest.importorskip('fastapi')

from mtrb_bringup.web_server import (  # noqa: E402, I100
    BroadcastHub, ClientChannel, SLOT_ORDER)


class Wakeup:
    def __init__(self):
        self.calls = 0

    def __call__(self):
        self.calls += 1


def test_channel_coalesces_by_kind_keeping_latest():
    channel = ClientChannel(Wakeup())
    channel.put('pose', 'p1')
    channel.put('pose', 'p2')
    channel.put('scan', 's1')
    assert channel.take_all() == ['p2', 's1']


def test_channel_take_all_empties_the_slots():
    channel = ClientChannel(Wakeup())
    channel.put('pose', 'p1')
    channel.take_all()
    assert channel.take_all() == []


def test_channel_orders_map_first_and_scan_last():
    channel = ClientChannel(Wakeup())
    for kind in reversed(SLOT_ORDER):
        channel.put(kind, kind if kind != 'map' else ('meta', b'bin'))
    assert channel.take_all() == ['meta', b'bin', 'error', 'plan', 'nav', 'status', 'pose', 'scan']


def test_slot_order_contains_nav_between_plan_and_status():
    assert SLOT_ORDER == ('map', 'error', 'plan', 'nav', 'status', 'pose', 'scan')


def test_channel_nav_coalesces_keeping_latest():
    channel = ClientChannel(Wakeup())
    channel.put('nav', 'n1')
    channel.put('nav', 'n2')
    channel.put('nav', 'n3')
    assert channel.take_all() == ['n3']


def test_channel_nav_is_sent_after_plan_and_before_status_and_pose():
    channel = ClientChannel(Wakeup())
    for kind in ('scan', 'pose', 'status', 'nav', 'plan'):
        channel.put(kind, kind)
    assert channel.take_all() == ['plan', 'nav', 'status', 'pose', 'scan']


def test_channel_nav_does_not_replace_other_kinds():
    channel = ClientChannel(Wakeup())
    channel.put('status', 's')
    channel.put('nav', 'n')
    channel.put('plan', 'p')
    assert sorted(channel.take_all()) == ['n', 'p', 's']


def test_hub_publish_nav_seeds_late_client_with_latest_value_only():
    hub = BroadcastHub()
    hub.publish('nav', 'active')
    hub.publish('nav', 'succeeded')
    channel = hub.add_client(Wakeup())
    assert channel.take_all() == ['succeeded']


def test_hub_publish_nav_reaches_every_client():
    hub = BroadcastHub()
    first, second = hub.add_client(Wakeup()), hub.add_client(Wakeup())
    hub.publish('nav', 'n')
    assert first.take_all() == ['n']
    assert second.take_all() == ['n']


def test_channel_unknown_kind_raises_key_error():
    channel = ClientChannel(Wakeup())
    with pytest.raises(KeyError):
        channel.put('bogus', 'x')


def test_channel_put_calls_wakeup():
    wakeup = Wakeup()
    ClientChannel(wakeup).put('pose', 'p')
    assert wakeup.calls == 1


def test_channel_put_on_closed_channel_is_ignored():
    wakeup = Wakeup()
    channel = ClientChannel(wakeup)
    channel.close()
    channel.put('pose', 'p')
    assert channel.take_all() == []
    assert wakeup.calls == 0


def test_channel_close_drops_pending_items():
    channel = ClientChannel(Wakeup())
    channel.put('pose', 'p')
    channel.close()
    assert channel.closed is True
    assert channel.take_all() == []


def test_channel_wakeup_runtime_error_marks_channel_closed():
    def dead_loop():
        raise RuntimeError('Event loop is closed')

    channel = ClientChannel(dead_loop)
    channel.put('pose', 'p')
    assert channel.closed is True
    channel.put('pose', 'p2')  # must not raise
    assert channel.closed is True


def test_channel_discard_removes_pending_and_tolerates_missing():
    channel = ClientChannel(Wakeup())
    channel.put('pose', 'p')
    channel.put('scan', 's')
    channel.discard('pose')
    channel.discard('plan')
    assert channel.take_all() == ['s']


def test_hub_publish_reaches_every_client():
    hub = BroadcastHub()
    a, b = hub.add_client(Wakeup()), hub.add_client(Wakeup())
    hub.publish('pose', 'p')
    assert a.take_all() == ['p']
    assert b.take_all() == ['p']


def test_hub_publish_unknown_kind_raises_key_error():
    with pytest.raises(KeyError):
        BroadcastHub().publish('bogus', 'x')


def test_hub_returns_none_when_full_and_does_not_register_it():
    hub = BroadcastHub(max_clients=2)
    assert hub.add_client(Wakeup()) is not None
    assert hub.add_client(Wakeup()) is not None
    assert hub.add_client(Wakeup()) is None
    assert hub.client_count == 2


def test_hub_remove_client_frees_a_slot_and_is_idempotent():
    hub = BroadcastHub(max_clients=1)
    channel = hub.add_client(Wakeup())
    hub.remove_client(channel)
    hub.remove_client(channel)
    assert hub.client_count == 0
    assert hub.add_client(Wakeup()) is not None


def test_hub_removed_client_receives_nothing():
    hub = BroadcastHub()
    channel = hub.add_client(Wakeup())
    hub.remove_client(channel)
    hub.publish('pose', 'p')
    assert channel.take_all() == []


def test_hub_seeds_new_client_with_latest_values():
    hub = BroadcastHub()
    hub.publish('pose', 'old')
    hub.publish('pose', 'new')
    hub.publish('map', ('meta', b'bin'))
    channel = hub.add_client(Wakeup())
    assert channel.take_all() == ['meta', b'bin', 'new']


def test_hub_discard_removes_latest_and_pending_of_clients():
    hub = BroadcastHub()
    existing = hub.add_client(Wakeup())
    hub.publish('scan', 's')
    hub.publish('pose', 'p')
    hub.discard('scan')
    assert existing.take_all() == ['p']
    assert hub.add_client(Wakeup()).take_all() == ['p']


def test_hub_discard_of_unknown_value_is_harmless():
    hub = BroadcastHub()
    hub.publish('pose', 'p')
    channel = hub.add_client(Wakeup())
    channel.take_all()
    hub.discard('scan')
    assert hub.client_count == 1
    assert channel.take_all() == []
    assert hub.add_client(Wakeup()).take_all() == ['p']  # other kinds are untouched


@pytest.mark.parametrize('bad', [0, -1, True, 1.5, '3', None])
def test_hub_rejects_invalid_max_clients(bad):
    with pytest.raises(ValueError):
        BroadcastHub(max_clients=bad)


def test_hub_slow_client_does_not_affect_others():
    hub = BroadcastHub()
    slow, fast = hub.add_client(Wakeup()), hub.add_client(Wakeup())
    for i in range(100):
        hub.publish('pose', f'p{i}')
    assert fast.take_all() == ['p99']
    assert slow.take_all() == ['p99']
