"""
Tests of TeleopGate (pure Python, injected clock, no sleep): task 18, criteria I-3 and A-4.

SAFETY-CRITICAL: the gate decides what the node publishes on cmd_vel_nav. Every test drives a
fake clock; times are binary-exact (multiples of 1/8 s) or built so that the float arithmetic
of the age/interval checks is exact.
"""

from mtrb_bringup.web_utils import (
    TELEOP_DROP, TELEOP_IDLE, TELEOP_MAX_RATE_HZ, TELEOP_MAX_SEQ, TELEOP_MAX_TRACKED_CLIENTS,
    TELEOP_NOTIFY_PERIOD_S, TELEOP_PUBLISH, TELEOP_REJECT, TELEOP_ZERO, TELEOP_ZERO_BURST,
    TeleopAction, TeleopGate)
import pytest

NAN = float('nan')
INF = float('inf')
TIMEOUT_S = 0.3


class Clock:
    """Fake clock: a callable returning `t` seconds."""

    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


def make_gate(clock=None, **kwargs):
    clock = Clock() if clock is None else clock
    params = {'max_vx': 0.2, 'max_wz': 0.8, 'timeout_s': TIMEOUT_S}
    params.update(kwargs)
    return TeleopGate(params.pop('max_vx'), params.pop('max_wz'), params.pop('timeout_s'),
                      clock, **params)


def msg(seq, vx=0.1, wz=0.2):
    return {'type': 'teleop', 'vx': vx, 'wz': wz, 'seq': seq}


def assert_zero_burst(action):
    assert action.kind == TELEOP_ZERO
    assert action.zeros == TELEOP_ZERO_BURST == 3
    assert action.twist is None


def assert_nothing(action, kind=TELEOP_IDLE):
    assert action.kind == kind
    assert action.zeros == 0
    assert action.twist is None


# --- constants and construction --------------------------------------------------------------

def test_safety_constants_have_the_specified_values():
    assert TELEOP_MAX_RATE_HZ == 30.0
    assert TELEOP_ZERO_BURST == 3
    assert TELEOP_MAX_TRACKED_CLIENTS == 64
    assert TELEOP_NOTIFY_PERIOD_S == 1.0


@pytest.mark.parametrize('kwargs', [
    {'max_vx': 0.0}, {'max_vx': -0.1}, {'max_vx': NAN}, {'max_vx': INF}, {'max_vx': True},
    {'max_vx': '0.2'}, {'max_wz': 0.0}, {'max_wz': -1.0}, {'max_wz': NAN},
    {'timeout_s': 0.0}, {'timeout_s': -0.3}, {'timeout_s': NAN}, {'timeout_s': INF},
    {'max_rate_hz': 0.0}, {'max_rate_hz': NAN}, {'notify_period_s': -1.0},
    {'zero_burst': 0}, {'zero_burst': True}, {'zero_burst': 1.5},
    {'enabled': 1}, {'enabled': None}])
def test_constructor_rejects_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        make_gate(**kwargs)


def test_constructor_rejects_a_non_callable_clock():
    with pytest.raises(ValueError):
        TeleopGate(0.2, 0.8, 0.3, 123.0)


def test_teleop_action_is_a_named_tuple_with_the_documented_fields():
    action = TeleopAction(TELEOP_ZERO, None, 3, None)
    assert (action.kind, action.twist, action.zeros, action.error) == (
        TELEOP_ZERO, None, 3, None)


# --- valid commands and saturation -----------------------------------------------------------

def test_valid_command_is_published_as_float_twist():
    action = make_gate().on_teleop('a', msg(0, vx=0.1, wz=-0.5), False)
    assert action.kind == TELEOP_PUBLISH
    assert action.twist == (0.1, -0.5)
    assert all(type(v) is float for v in action.twist)
    assert action.zeros == 0
    assert action.error is None


def test_integer_velocities_are_accepted_and_returned_as_floats():
    action = make_gate().on_teleop('a', msg(0, vx=0, wz=0), False)
    assert action.kind == TELEOP_PUBLISH
    assert action.twist == (0.0, 0.0)
    assert all(type(v) is float for v in action.twist)


@pytest.mark.parametrize('vx, wz, expected', [
    (5.0, 9.0, (0.2, 0.8)),
    (-5.0, -9.0, (-0.2, -0.8)),
    (0.2, 0.8, (0.2, 0.8)),
    (-0.2, -0.8, (-0.2, -0.8)),
    (0.2000001, 0.8000001, (0.2, 0.8)),
    (1e308, -1e308, (0.2, -0.8)),
    (0.05, -0.3, (0.05, -0.3))])
def test_velocities_are_saturated_to_the_default_limits(vx, wz, expected):
    action = make_gate().on_teleop('a', msg(0, vx=vx, wz=wz), False)
    assert action.kind == TELEOP_PUBLISH
    assert action.twist == expected


@pytest.mark.parametrize('max_vx, max_wz', [(0.46, 1.9), (0.1, 0.3), (0.01, 0.01)])
def test_velocities_are_saturated_to_other_configured_limits(max_vx, max_wz):
    gate = make_gate(max_vx=max_vx, max_wz=max_wz)
    action = gate.on_teleop('a', msg(0, vx=10.0, wz=-10.0), False)
    assert action.twist == (max_vx, -max_wz)


def test_saturation_never_exceeds_the_limit_for_any_sign_combination():
    for vx, wz in [(9, 9), (9, -9), (-9, 9), (-9, -9)]:
        twist = make_gate().on_teleop('a', msg(0, vx=vx, wz=wz), False).twist
        assert abs(twist[0]) == 0.2 and abs(twist[1]) == 0.8
        assert (twist[0] > 0) == (vx > 0) and (twist[1] > 0) == (wz > 0)


# --- invalid messages ------------------------------------------------------------------------

@pytest.mark.parametrize('bad', [NAN, INF, -INF, True, False, '0.1', None, [0.1], {'a': 1}])
@pytest.mark.parametrize('field', ['vx', 'wz'])
def test_non_finite_or_non_numeric_velocity_is_rejected_without_publishing(field, bad):
    message = msg(0)
    message[field] = bad
    gate = make_gate()
    action = gate.on_teleop('a', message, False)
    assert_nothing(action, TELEOP_REJECT)
    assert action.error[0] == 'invalid_teleop'
    assert gate.active is False  # an invalid message never takes the lease


@pytest.mark.parametrize('field', ['vx', 'wz', 'seq'])
def test_missing_field_is_rejected(field):
    message = msg(0)
    del message[field]
    action = make_gate().on_teleop('a', message, False)
    assert_nothing(action, TELEOP_REJECT)
    assert action.error[0] == 'invalid_teleop'


@pytest.mark.parametrize('bad_seq', [True, False, 1.5, 1.0, -1, '1', None, [1],
                                     TELEOP_MAX_SEQ + 1, NAN, INF])
def test_invalid_seq_is_rejected(bad_seq):
    action = make_gate().on_teleop('a', msg(bad_seq), False)
    assert_nothing(action, TELEOP_REJECT)
    assert action.error[0] == 'invalid_teleop'


@pytest.mark.parametrize('seq', [0, 1, TELEOP_MAX_SEQ])
def test_seq_bounds_are_accepted(seq):
    assert make_gate().on_teleop('a', msg(seq), False).kind == TELEOP_PUBLISH


@pytest.mark.parametrize('message', [None, [], [1, 2], 'teleop', 5, 1.5, True, (1, 2)])
def test_message_that_is_not_a_dict_is_rejected(message):
    action = make_gate().on_teleop('a', message, False)
    assert_nothing(action, TELEOP_REJECT)
    assert action.error[0] == 'invalid_teleop'


def test_invalid_message_does_not_refresh_the_watchdog():
    clock = Clock(0.0)
    gate = make_gate(clock)
    assert gate.on_teleop('a', msg(0), False).kind == TELEOP_PUBLISH
    clock.t = 0.25
    assert gate.on_teleop('a', msg(1, vx=NAN), False).kind == TELEOP_REJECT
    clock.t = 0.3125
    assert_zero_burst(gate.watchdog_tick())


def test_invalid_message_of_the_owner_does_not_release_the_lease():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.125
    gate.on_teleop('a', msg(1, vx=NAN), False)
    assert gate.active is True
    assert gate.on_teleop('b', msg(0), False).error[0] == 'teleop_rejected'


# --- sequence numbers and rate limit ---------------------------------------------------------

@pytest.mark.parametrize('second_seq', [5, 4, 0])
def test_repeated_or_older_seq_is_dropped_silently(second_seq):
    clock = Clock(0.0)
    gate = make_gate(clock)
    assert gate.on_teleop('a', msg(5), False).kind == TELEOP_PUBLISH
    clock.t = 0.125
    action = gate.on_teleop('a', msg(second_seq), False)
    assert_nothing(action, TELEOP_DROP)
    assert action.error is None


def test_a_replayed_seq_does_not_keep_the_robot_moving():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(5), False)
    clock.t = 0.25
    assert gate.on_teleop('a', msg(5), False).kind == TELEOP_DROP
    clock.t = 0.3125  # 312.5 ms after the only valid command
    assert_zero_burst(gate.watchdog_tick())


def test_higher_seq_is_accepted_and_gaps_are_fine():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(1), False)
    clock.t = 0.125
    assert gate.on_teleop('a', msg(100), False).kind == TELEOP_PUBLISH


def test_seq_is_tracked_per_client():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(50), False)
    clock.t = 0.5  # a's lease expired
    assert gate.on_teleop('b', msg(0), False).kind == TELEOP_PUBLISH


def test_seq_restarts_after_the_client_disconnected():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(50), False)
    gate.client_disconnected('a')
    clock.t = 0.125
    assert gate.on_teleop('a', msg(0), False).kind == TELEOP_PUBLISH


def test_message_faster_than_30_hz_is_dropped_silently():
    clock = Clock(0.0)
    gate = make_gate(clock)
    assert gate.on_teleop('a', msg(0), False).kind == TELEOP_PUBLISH
    clock.t = 1.0 / 30.0 - 0.001
    action = gate.on_teleop('a', msg(1), False)
    assert_nothing(action, TELEOP_DROP)
    assert action.error is None


def test_message_at_exactly_30_hz_is_accepted():
    clock = Clock(0.0)
    gate = make_gate(clock)
    assert gate.on_teleop('a', msg(0), False).kind == TELEOP_PUBLISH
    clock.t = 1.0 / 30.0
    assert gate.on_teleop('a', msg(1), False).kind == TELEOP_PUBLISH


def test_message_at_the_same_instant_is_dropped():
    gate = make_gate()
    gate.on_teleop('a', msg(0), False)
    assert gate.on_teleop('a', msg(1), False).kind == TELEOP_DROP


def test_a_rate_limited_message_does_not_refresh_the_watchdog():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.015625
    assert gate.on_teleop('a', msg(1), False).kind == TELEOP_DROP
    clock.t = 0.3125
    assert_zero_burst(gate.watchdog_tick())


def test_a_dropped_message_does_not_postpone_the_next_accepted_one():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.015625
    gate.on_teleop('a', msg(1), False)  # dropped
    clock.t = 1.0 / 30.0
    assert gate.on_teleop('a', msg(2), False).kind == TELEOP_PUBLISH


def test_rate_limit_is_per_client_not_global():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    gate.client_disconnected('a')
    assert gate.on_teleop('b', msg(0), False).kind == TELEOP_PUBLISH


def test_rate_limit_uses_the_configured_rate():
    clock = Clock(0.0)
    gate = make_gate(clock, max_rate_hz=8.0)  # 0.125 s
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.0625
    assert gate.on_teleop('a', msg(1), False).kind == TELEOP_DROP
    clock.t = 0.125
    assert gate.on_teleop('a', msg(1), False).kind == TELEOP_PUBLISH


# --- watchdog (I-3) --------------------------------------------------------------------------

def test_watchdog_is_idle_when_nobody_controls():
    assert_nothing(make_gate().watchdog_tick())


def test_watchdog_does_not_expire_at_exactly_300_ms():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.3
    assert_nothing(gate.watchdog_tick())
    assert gate.active is True


def test_watchdog_expires_just_after_300_ms_with_a_zero_burst():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.3001
    assert_zero_burst(gate.watchdog_tick())
    assert gate.active is False


def test_watchdog_burst_is_followed_by_idle_forever():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.4
    assert_zero_burst(gate.watchdog_tick())
    for step in range(1, 200):
        clock.t = 0.4 + step * 0.02
        assert_nothing(gate.watchdog_tick())


def test_watchdog_timeout_follows_the_configuration():
    clock = Clock(0.0)
    gate = make_gate(clock, timeout_s=0.05)
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.05
    assert_nothing(gate.watchdog_tick())
    clock.t = 0.0501
    assert_zero_burst(gate.watchdog_tick())


def test_each_valid_command_extends_the_deadline():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.25
    gate.on_teleop('a', msg(1), False)
    clock.t = 0.5  # 250 ms after the last valid one
    assert_nothing(gate.watchdog_tick())
    clock.t = 0.5626
    assert_zero_burst(gate.watchdog_tick())


def test_clock_going_backwards_counts_as_expired():
    clock = Clock(10.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 9.0
    assert gate.active is False
    assert_zero_burst(gate.watchdog_tick())


def test_clock_going_backwards_lets_another_client_take_over_only_after_the_zero_burst_state():
    clock = Clock(10.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 9.0  # stale lease: not fresh
    assert gate.on_teleop('b', msg(0), False).kind == TELEOP_PUBLISH


def test_owner_age_is_none_without_owner_and_grows_with_the_clock():
    clock = Clock(1.0)
    gate = make_gate(clock)
    assert gate.owner_age_s is None
    gate.on_teleop('a', msg(0), False)
    clock.t = 1.125
    assert gate.owner_age_s == pytest.approx(0.125)
    gate.stop()
    assert gate.owner_age_s is None


# --- lease: one controller at a time ---------------------------------------------------------

def test_other_client_is_rejected_while_the_owner_is_fresh():
    clock = Clock(0.0)
    gate = make_gate(clock)
    assert gate.on_teleop('a', msg(0), False).kind == TELEOP_PUBLISH
    clock.t = 0.125
    action = gate.on_teleop('b', msg(0), False)
    assert_nothing(action, TELEOP_REJECT)
    assert action.error[0] == 'teleop_rejected'
    assert gate.active is True


def test_rejected_client_does_not_disturb_the_owner():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.125
    gate.on_teleop('b', msg(0), False)
    clock.t = 0.25
    assert gate.on_teleop('a', msg(1), False).kind == TELEOP_PUBLISH


def test_other_client_takes_control_when_the_owner_expired():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.3001
    action = gate.on_teleop('b', msg(0), False)
    assert action.kind == TELEOP_PUBLISH
    assert action.error is None
    clock.t = 0.375
    assert gate.on_teleop('a', msg(1), False).error[0] == 'teleop_rejected'


def test_other_client_is_still_rejected_at_exactly_300_ms():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.3
    assert gate.on_teleop('b', msg(0), False).kind == TELEOP_REJECT


def test_other_client_takes_control_after_the_owner_disconnected():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    assert_zero_burst(gate.client_disconnected('a'))
    clock.t = 0.0625
    assert gate.on_teleop('b', msg(0), False).kind == TELEOP_PUBLISH


def test_disconnect_of_a_non_owner_keeps_the_owner_and_orders_nothing():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    assert_nothing(gate.client_disconnected('b'))
    assert gate.active is True
    clock.t = 0.125
    assert gate.on_teleop('a', msg(1), False).kind == TELEOP_PUBLISH


def test_disconnect_of_an_unknown_client_is_harmless():
    assert_nothing(make_gate().client_disconnected('nobody'))


def test_disconnect_twice_only_orders_one_zero_burst():
    gate = make_gate()
    gate.on_teleop('a', msg(0), False)
    assert_zero_burst(gate.client_disconnected('a'))
    assert_nothing(gate.client_disconnected('a'))


# --- error reply throttle --------------------------------------------------------------------

def test_error_reply_is_sent_once_per_second_per_client():
    clock = Clock(0.0)
    gate = make_gate(clock)
    assert gate.on_teleop('a', msg(0), True).error[0] == 'teleop_rejected'
    clock.t = 0.5
    silent = gate.on_teleop('a', msg(1), True)
    assert silent.kind == TELEOP_REJECT and silent.error is None
    clock.t = 0.984375
    assert gate.on_teleop('a', msg(2), True).error is None
    clock.t = 1.0
    assert gate.on_teleop('a', msg(3), True).error is not None


def test_error_throttle_is_independent_per_client():
    clock = Clock(0.0)
    gate = make_gate(clock)
    assert gate.on_teleop('a', msg(0), True).error is not None
    clock.t = 0.25
    assert gate.on_teleop('b', msg(0), True).error is not None
    assert gate.on_teleop('a', msg(1), True).error is None


def test_error_is_sent_again_if_the_clock_goes_backwards():
    clock = Clock(5.0)
    gate = make_gate(clock)
    assert gate.on_teleop('a', msg(0), True).error is not None
    clock.t = 4.0
    assert gate.on_teleop('a', msg(1), True).error is not None


def test_invalid_message_errors_are_throttled_too():
    clock = Clock(0.0)
    gate = make_gate(clock)
    assert gate.on_teleop('a', None, False).error[0] == 'invalid_teleop'
    clock.t = 0.25
    action = gate.on_teleop('a', None, False)
    assert action.kind == TELEOP_REJECT and action.error is None


def test_throttled_rejection_still_orders_the_zeros_of_a_busy_owner():
    clock = Clock(0.0)
    gate = make_gate(clock)
    assert gate.on_teleop('a', msg(0), True).error is not None  # busy, no owner yet
    clock.t = 0.125
    gate.on_teleop('a', msg(1), False)                          # a becomes the owner
    clock.t = 0.25
    action = gate.on_teleop('a', msg(2), True)                  # busy again, error throttled
    assert_zero_burst(action)
    assert action.error is None


# --- navigation busy -------------------------------------------------------------------------

def test_busy_navigation_rejects_the_command_without_zeros_when_nobody_controls():
    action = make_gate().on_teleop('a', msg(0), True)
    assert_nothing(action, TELEOP_REJECT)
    assert action.error[0] == 'teleop_rejected'


def test_busy_navigation_with_an_active_owner_orders_zeros_and_rejects():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.125
    action = gate.on_teleop('a', msg(1), True)
    assert_zero_burst(action)
    assert action.error[0] == 'teleop_rejected'
    assert gate.active is False
    assert_nothing(gate.watchdog_tick())  # no second burst from the watchdog


def test_busy_navigation_triggered_by_another_client_also_stops_the_owner():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    clock.t = 0.125
    action = gate.on_teleop('b', msg(0), True)
    assert_zero_burst(action)
    assert gate.active is False


def test_nothing_is_published_while_navigation_stays_busy():
    clock = Clock(0.0)
    gate = make_gate(clock)
    for i in range(10):
        clock.t = i * 0.125
        action = gate.on_teleop('a', msg(i), True)
        assert action.kind == TELEOP_REJECT
        assert action.twist is None


def test_command_is_accepted_again_when_navigation_is_no_longer_busy():
    clock = Clock(0.0)
    gate = make_gate(clock)
    assert gate.on_teleop('a', msg(0), True).kind == TELEOP_REJECT
    clock.t = 0.125
    assert gate.on_teleop('a', msg(1), False).kind == TELEOP_PUBLISH


# --- disabled --------------------------------------------------------------------------------

def test_disabled_gate_rejects_every_valid_command_as_teleop_disabled():
    gate = make_gate(enabled=False)
    action = gate.on_teleop('a', msg(0), False)
    assert_nothing(action, TELEOP_REJECT)
    assert action.error[0] == 'teleop_disabled'
    assert gate.active is False


def test_disabled_gate_has_priority_over_busy_and_invalid_messages():
    gate = make_gate(enabled=False)
    assert gate.on_teleop('a', msg(0), True).error[0] == 'teleop_disabled'
    assert gate.on_teleop('b', None, False).error[0] == 'teleop_disabled'


def test_disabled_gate_still_honours_stop():
    assert_zero_burst(make_gate(enabled=False).stop())


# --- stop ------------------------------------------------------------------------------------

def test_stop_without_owner_still_orders_zeros():
    assert_zero_burst(make_gate().stop())


def test_stop_ends_the_teleop_and_the_watchdog_stays_quiet():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    assert_zero_burst(gate.stop())
    assert gate.active is False
    clock.t = 5.0
    assert_nothing(gate.watchdog_tick())


def test_stop_twice_orders_zeros_both_times():
    gate = make_gate()
    assert_zero_burst(gate.stop())
    assert_zero_burst(gate.stop())


def test_stop_is_not_subject_to_rate_or_notify_limits():
    gate = make_gate()
    for _ in range(50):
        assert_zero_burst(gate.stop())


def test_stop_does_not_reopen_the_replay_window():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(5), False)
    gate.stop()
    clock.t = 0.125
    assert gate.on_teleop('a', msg(5), False).kind == TELEOP_DROP
    assert gate.on_teleop('a', msg(6), False).kind == TELEOP_PUBLISH


def test_another_client_can_take_control_right_after_a_stop():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(0), False)
    gate.stop()
    clock.t = 0.0625
    assert gate.on_teleop('b', msg(0), False).kind == TELEOP_PUBLISH


def test_zero_burst_size_is_configurable():
    gate = make_gate(zero_burst=5)
    assert gate.stop().zeros == 5


# --- bounded tables --------------------------------------------------------------------------

def test_per_client_tables_never_grow_beyond_the_bound():
    clock = Clock(0.0)
    gate = make_gate(clock)
    for i in range(TELEOP_MAX_TRACKED_CLIENTS * 3):
        clock.t = i * 0.5  # each client takes over after the previous lease expired
        assert gate.on_teleop(f'c{i}', msg(0), False).kind == TELEOP_PUBLISH
        gate.on_teleop(f'c{i}', msg(0), True)  # busy reject also feeds the notify table
    # Private tables on purpose: memory growth has no public observable.
    assert len(gate._last_seq) <= TELEOP_MAX_TRACKED_CLIENTS
    assert len(gate._last_accept_s) <= TELEOP_MAX_TRACKED_CLIENTS
    assert len(gate._last_notify_s) <= TELEOP_MAX_TRACKED_CLIENTS


def test_notify_table_is_bounded_for_rejected_clients():
    gate = make_gate()
    for i in range(TELEOP_MAX_TRACKED_CLIENTS * 4):
        gate.on_teleop(f'spam{i}', msg(0), True)
    assert len(gate._last_notify_s) <= TELEOP_MAX_TRACKED_CLIENTS


def test_the_current_owner_is_never_evicted_from_the_tables():
    clock = Clock(0.0)
    gate = make_gate(clock)
    for i in range(TELEOP_MAX_TRACKED_CLIENTS * 2):
        clock.t = i * 0.5
        gate.on_teleop(f'c{i}', msg(7), False)
    owner = f'c{TELEOP_MAX_TRACKED_CLIENTS * 2 - 1}'
    clock.t += 0.125
    # still the owner with its seq remembered: a replayed seq is dropped, not published
    assert gate.on_teleop(owner, msg(7), False).kind == TELEOP_DROP
    assert gate.on_teleop(owner, msg(8), False).kind == TELEOP_PUBLISH


# --- state machine sanity --------------------------------------------------------------------

def test_active_reflects_the_lease_state():
    clock = Clock(0.0)
    gate = make_gate(clock)
    assert gate.active is False
    gate.on_teleop('a', msg(0), False)
    assert gate.active is True
    clock.t = 0.3
    assert gate.active is True
    clock.t = 0.3001
    assert gate.active is False


def test_full_cycle_move_silence_zero_burst_then_idle_then_move_again():
    clock = Clock(0.0)
    gate = make_gate(clock)
    published = []
    for seq in range(4):
        clock.t = seq * 0.125
        published.append(gate.on_teleop('a', msg(seq), False).kind)
    assert published == [TELEOP_PUBLISH] * 4
    kinds = []
    for step in range(1, 40):
        clock.t = 0.375 + step * 0.0625
        kinds.append(gate.watchdog_tick())
    zero_bursts = [a for a in kinds if a.kind == TELEOP_ZERO]
    assert len(zero_bursts) == 1 and zero_bursts[0].zeros == 3
    assert all(a.kind in (TELEOP_IDLE, TELEOP_ZERO) for a in kinds)
    clock.t += 0.125
    assert gate.on_teleop('a', msg(4), False).kind == TELEOP_PUBLISH


# --- phase 4: huge integers and the default clock --------------------------------------------
# An int like 10**400 cannot become a float (OverflowError). It must be refused as an invalid
# command, never raise, never publish and never take the lease.

HUGE = 10 ** 400


@pytest.mark.parametrize('field', ['vx', 'wz'])
@pytest.mark.parametrize('huge', [HUGE, -HUGE])
def test_huge_integer_velocity_is_invalid_teleop_without_exception(field, huge):
    gate = make_gate()
    message = msg(0)
    message[field] = huge
    action = gate.on_teleop('a', message, False)
    assert action.kind == TELEOP_REJECT
    assert action.error is not None and action.error[0] == 'invalid_teleop'
    assert action.twist is None and action.zeros == 0
    assert gate.owner_age_s is None and not gate.active


def test_huge_integer_velocity_does_not_take_the_lease_nor_block_others():
    gate = make_gate()
    gate.on_teleop('a', msg(0, vx=HUGE), False)
    assert gate.on_teleop('b', msg(0), False).kind == TELEOP_PUBLISH


def test_huge_integer_velocity_of_the_owner_does_not_refresh_its_lease():
    clock = Clock(0.0)
    gate = make_gate(clock)
    assert gate.on_teleop('a', msg(0), False).kind == TELEOP_PUBLISH
    clock.t = 0.25
    assert gate.on_teleop('a', msg(1, wz=HUGE), False).kind == TELEOP_REJECT
    clock.t = 0.3125
    assert gate.watchdog_tick().kind == TELEOP_ZERO     # expired from the last VALID command


def test_huge_integer_seq_is_invalid_teleop_without_exception():
    gate = make_gate()
    action = gate.on_teleop('a', msg(HUGE), False)
    assert action.kind == TELEOP_REJECT
    assert action.error is not None and action.error[0] == 'invalid_teleop'
    assert action.twist is None and not gate.active


def test_huge_integer_seq_is_rejected_silently_when_the_client_was_notified_just_before():
    clock = Clock(0.0)
    gate = make_gate(clock)
    assert gate.on_teleop('a', msg(HUGE), False).error is not None
    clock.t = 0.125                                     # inside TELEOP_NOTIFY_PERIOD_S
    action = gate.on_teleop('a', msg(HUGE), False)
    assert action.kind == TELEOP_REJECT
    assert action.error is None
    assert action.twist is None


def test_a_huge_seq_does_not_poison_the_sequence_of_the_client():
    clock = Clock(0.0)
    gate = make_gate(clock)
    gate.on_teleop('a', msg(HUGE), False)
    clock.t = 0.125
    assert gate.on_teleop('a', msg(0), False).kind == TELEOP_PUBLISH


def test_the_default_clock_of_the_gate_is_time_monotonic():
    import inspect
    import time
    default = inspect.signature(TeleopGate.__init__).parameters['clock'].default
    assert default is time.monotonic


def test_a_gate_without_an_injected_clock_works_with_the_real_monotonic_time():
    gate = TeleopGate(0.2, 0.8, 5.0)
    assert gate.on_teleop('a', msg(0), False).kind == TELEOP_PUBLISH
    assert gate.active
    age = gate.owner_age_s
    assert 0.0 <= age < 5.0
    assert gate.watchdog_tick().kind == TELEOP_IDLE
    assert gate.stop().kind == TELEOP_ZERO and not gate.active
