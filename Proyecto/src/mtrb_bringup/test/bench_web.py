"""
Load/CPU benchmark of the pure components of the web bridge (task 22, criteria A-5/A-6).

It has no ROS dependency and its name has no `test_` prefix on purpose: pytest and
`colcon test` do not collect it. It measures with `time.perf_counter` (time per call),
`tracemalloc` (peak Python memory of one run) and `resource` (peak RSS), repeating each
case N times and reporting mean / p95 / max, and prints a Markdown table with the share of
ONE core that each operation needs at its real rate (time per cycle x frequency).

It is isolated (`python -I` ignores PYTHONPATH and the current directory), so the package
folder is added to `sys.path` from the location of this file. It needs `fastapi` and
`pyyaml` only because `mtrb_bringup.web_server` imports them (use the project venv).

How to run (from `Proyecto/src/mtrb_bringup`)::

    ../../.venv/bin/python -I test/bench_web.py                 # full run (about 1-3 min)
    ../../.venv/bin/python -I test/bench_web.py --quick         # short run (about 1-2 s)
    ../../.venv/bin/python -I test/bench_web.py --only maps,scan --repeat 5
    ../../.venv/bin/python -I test/bench_web.py --only maps --profile   # cProfile
    ../../.venv/bin/python -I test/bench_web.py --real-hub 5    # + 5 s with real threads

Groups for `--only` (comma separated): maps, scan, path, hub, teleop, listing.
The Raspberry Pi 5 columns are an explicit HYPOTHESIS (this machine x3 and x5), not a
measurement: the real figures must be taken on the robot (see test/BENCHMARKS.md).
"""

import argparse
import array
import asyncio
import cProfile
import hashlib
import io
import math
import os
import platform
import pstats
import random
import resource
import sys
import tempfile
import threading
import time
import tracemalloc

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mtrb_bringup.web_server import BroadcastHub, json_message, MapSaver  # noqa: E402
from mtrb_bringup.web_utils import (  # noqa: E402
    downsample_scan, list_maps, occupancy_to_bytes, pack_map_binary, scan_to_map_points,
    simplify_path, TeleopGate)

GROUPS = ('maps', 'scan', 'path', 'hub', 'teleop', 'listing')
PI_SLOWDOWNS = (3.0, 5.0)         # HYPOTHESIS: Pi 5 is 3-5 times slower than this machine
CPU_LIMIT = 0.70                  # A-5: the Pi must stay below 70 % CPU
MAP_SIDES = (500, 1000, 2000, 4000)
SCAN_BEAMS = (360, 720, 1800)
SCAN_MAX_POINTS = 360             # param/web.yaml scan.max_points
PLAN_MAX_POINTS = 200             # param/web.yaml plan.max_points
RATES = {'pose': 10.0, 'scan': 5.0, 'status': 1.0, 'map': 1.0, 'plan': 1.0,
         'teleop': 30.0, 'watchdog': 50.0, 'listing': 1.0}


def percentile(values, fraction):
    """Return the nearest-rank percentile of a non-empty list; fraction is in (0, 1]."""
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


class Row:
    """One line of the result table: timing statistics of a case, in seconds."""

    def __init__(self, group, name, detail, samples, rate_hz, in_total=False, extra=''):
        """Store the samples (seconds per cycle) and derive mean/p95/max."""
        self.group = group
        self.name = name
        self.detail = detail
        self.rate_hz = rate_hz
        self.in_total = in_total
        self.extra = extra
        self.count = len(samples)
        self.mean = sum(samples) / len(samples)
        self.p95 = percentile(samples, 0.95)
        self.maximum = max(samples)

    def core_share(self, slowdown=1.0):
        """Fraction of one core used at `rate_hz` if the machine is `slowdown` times slower."""
        return self.mean * self.rate_hz * slowdown

    def exceeded(self):
        """Return the Pi slowdowns (hypothesis) for which this row alone passes 70 %."""
        return [s for s in PI_SLOWDOWNS if self.core_share(s) > CPU_LIMIT]


def time_calls(function, repeat):
    """Call `function()` `repeat` times and return the elapsed seconds of each call."""
    samples = []
    for _ in range(repeat):
        start = time.perf_counter()
        function()
        samples.append(time.perf_counter() - start)
    return samples


def peak_memory_mb(function):
    """Run `function()` once under tracemalloc and return its peak Python memory in MB."""
    tracemalloc.start()
    try:
        function()
        return tracemalloc.get_traced_memory()[1] / (1024 * 1024)
    finally:
        tracemalloc.stop()


def peak_rss_mb():
    """Return the peak resident set size of this process in MB."""
    value = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return value / (1024 * 1024) if sys.platform == 'darwin' else value / 1024


def fmt_ms(seconds):
    """Format seconds as milliseconds with precision adapted to the magnitude."""
    ms = seconds * 1000.0
    if ms >= 100:
        return f'{ms:.0f}'
    if ms >= 1:
        return f'{ms:.2f}'
    return f'{ms:.4f}'


# --- (a) occupancy map pipeline ---------------------------------------------------------
def make_grid(side, seed=1):
    """Return a pseudo-realistic side x side occupancy grid as bytes (int8: -1, 0, 100)."""
    rng = random.Random(seed)
    unknown, free, occupied = b'\xff', b'\x00', b'\x64'
    margin = side // 10
    inner = side - 2 * margin
    free_row = unknown * margin + occupied * 2 + free * (inner - 4) + occupied * 2 \
        + unknown * margin
    wall_row = unknown * margin + occupied * inner + unknown * margin
    unknown_row = unknown * side
    rows = []
    for y in range(side):
        if y < margin or y >= side - margin:
            row = bytearray(unknown_row)
        elif y in (margin, side - margin - 1) or (y - margin) % max(8, inner // 5) == 0:
            row = bytearray(wall_row)
            for door in range(3):  # door gaps in the inner walls
                start = margin + 2 + (door * inner) // 3 + rng.randrange(max(1, inner // 6))
                row[start:start + 6] = free * 6
        else:
            row = bytearray(free_row)
            for column in range(margin + inner // 4, margin + inner, max(8, inner // 4)):
                row[column] = 100  # pillars / inner wall columns
            for _ in range(2):  # sensor noise: isolated occupied or unknown cells
                row[rng.randrange(margin, side - margin)] = rng.choice((100, 255))
        rows.append(bytes(row))
    return b''.join(rows)


def map_pipeline(data, side):
    """Run the map encoding as the node does (hash only for buffers, then zlib)."""
    if isinstance(data, array.array):
        hashlib.blake2b(bytes(memoryview(data).cast('B')), digest_size=16).digest()
    return pack_map_binary(1, occupancy_to_bytes(data, side, side))


def bench_maps(repeat, sides):
    """Time hash + occupancy_to_bytes + pack_map_binary for several map sizes."""
    rows = []
    for side in sides:
        raw = make_grid(side)
        cases = (('array(b)', array.array('b', raw)),
                 ('list', array.array('b', raw).tolist()))
        for kind, data in cases:
            count = max(2, repeat if side <= 1000 else (repeat + 1) // 2)
            steps = {'hash': [], 'encode': [], 'pack': []}
            compressed = b''
            for _ in range(count):
                if kind == 'array(b)':  # production hashes the buffer of the array
                    start = time.perf_counter()
                    hashlib.blake2b(bytes(memoryview(data).cast('B')), digest_size=16).digest()
                    steps['hash'].append(time.perf_counter() - start)
                start = time.perf_counter()
                compressed = occupancy_to_bytes(data, side, side)
                steps['encode'].append(time.perf_counter() - start)
                start = time.perf_counter()
                pack_map_binary(1, compressed)
                steps['pack'].append(time.perf_counter() - start)
            totals = [e + p for e, p in zip(steps['encode'], steps['pack'])]
            if steps['hash']:
                totals = [t + h for t, h in zip(totals, steps['hash'])]
            hash_text = 'n/a'
            if steps['hash']:
                hash_text = fmt_ms(sum(steps['hash']) / len(steps['hash'])) + ' ms'
            encode_ms = fmt_ms(sum(steps['encode']) / count)
            mem = peak_memory_mb(lambda d=data, s=side: map_pipeline(d, s))
            extra = (f'hash {hash_text}; encode {encode_ms} ms; '
                     f'zlib {len(compressed) / 1024:.1f} KB '
                     f'({len(raw) / max(1, len(compressed)):.0f}:1); '
                     f'peak mem {mem:.0f} MB')
            rows.append(Row('maps', f'map {side}x{side}', kind, totals, RATES['map'],
                            in_total=(side == 1000 and kind == 'array(b)'), extra=extra))
    return rows


# --- (b) scan pipeline ------------------------------------------------------------------
def make_ranges(beams, seed=2):
    """Return `beams` ranges (m) of a room: walls at 0.5-11 m, ~3 % inf, ~2 % out of range."""
    rng = random.Random(seed)
    ranges = []
    for index in range(beams):
        roll = rng.random()
        if roll < 0.03:
            ranges.append(float('inf'))
        elif roll < 0.05:
            ranges.append(30.0)
        else:
            ranges.append(5.0 + 4.0 * math.sin(6.0 * index / beams) + rng.random() * 0.05)
    return ranges


def bench_scan(repeat):
    """Time downsample_scan + scan_to_map_points + JSON for 360, 720 and 1800 beams."""
    rows = []
    pose = (1.25, -3.5, 0.7)
    for beams in SCAN_BEAMS:
        for kind in ('array(f)', 'list'):
            plain = make_ranges(beams)
            ranges = array.array('f', plain) if kind == 'array(f)' else plain
            increment = 2.0 * math.pi / beams
            steps = {'downsample': [], 'project': [], 'json': []}
            for _ in range(repeat):
                start = time.perf_counter()
                points = downsample_scan(ranges, -math.pi, increment, SCAN_MAX_POINTS)
                steps['downsample'].append(time.perf_counter() - start)
                start = time.perf_counter()
                flat = scan_to_map_points(points, pose)
                steps['project'].append(time.perf_counter() - start)
                start = time.perf_counter()
                text = json_message('scan', stamp=1.0, points=flat)
                steps['json'].append(time.perf_counter() - start)
            totals = [sum(parts) for parts in zip(*steps.values())]
            extra = ('; '.join(f'{k} {fmt_ms(sum(v) / len(v))} ms' for k, v in steps.items())
                     + f'; message {len(text) / 1024:.1f} KB')
            rows.append(Row('scan', f'scan {beams} beams', kind, totals, RATES['scan'],
                            in_total=(beams == 720 and kind == 'array(f)'), extra=extra))
    return rows


# --- (c) path ---------------------------------------------------------------------------
def bench_path(repeat):
    """Time simplify_path for a 2000-point plan."""
    path = [[i * 0.05, math.sin(i / 100.0)] for i in range(2000)]
    samples = time_calls(lambda: simplify_path(path, PLAN_MAX_POINTS), repeat)
    return [Row('path', 'simplify_path', '2000 -> 200 points', samples, RATES['plan'],
                in_total=True)]


# --- (d) BroadcastHub -------------------------------------------------------------------
def hub_payloads(map_side):
    """Return realistic payloads: pose, scan, status JSON and the (meta, binary) of a map."""
    flat = scan_to_map_points(downsample_scan(make_ranges(720), -math.pi, 2 * math.pi / 720,
                                              SCAN_MAX_POINTS), (1.0, 2.0, 0.5))
    nodes = {n: 'active' for n in ('controller_server', 'planner_server', 'bt_navigator',
                                   'behavior_server', 'velocity_smoother', 'slam_toolbox')}
    binary = pack_map_binary(
        1, occupancy_to_bytes(array.array('b', make_grid(map_side)), map_side, map_side))
    meta = json_message('map_meta', version=1, width=map_side, height=map_side,
                        resolution=0.05, origin_x=-25.0, origin_y=-25.0)
    return {
        'pose': json_message('pose', x=1.2345, y=-2.3456, yaw=0.1234, stamp=1234.567),
        'scan': json_message('scan', stamp=1234.567, points=flat),
        'status': json_message('status', battery_pct=87.5, mode='mapping', nodes=nodes,
                               scan_age=0.12, odom_age=0.03,
                               dock={'x': 0.0, 'y': 0.0, 'yaw': 0.0}),
        'map': (meta, binary)}


def payload_bytes(payload):
    """Return the size in bytes of a str or (str, bytes) payload."""
    items = payload if isinstance(payload, tuple) else (payload,)
    return sum(len(i) for i in items)


def timeline(duration_s):
    """Return the sorted (time_s, kind) publications of `duration_s` seconds."""
    events = []
    for kind, period, offset in (('pose', 0.1, 0.0), ('scan', 0.2, 0.05),
                                 ('status', 1.0, 0.3), ('map', 1.0, 0.5)):
        count = int(duration_s / period + 1e-9)
        events.extend((round(offset + i * period, 6), kind) for i in range(count))
    return sorted(events)


def bench_hub(duration_s, map_side, slow_period_s=2.0, clients=5):
    """Simulate `clients` clients (one slow) for `duration_s` seconds of simulated time."""
    payloads = hub_payloads(map_side)
    hub = BroadcastHub(max_clients=clients)
    channels = [hub.add_client(lambda: None) for _ in range(clients)]
    slow = channels[-1]
    publish_cost = {kind: [] for kind in payloads}
    published = {kind: 0 for kind in payloads}
    delivered = {kind: 0 for kind in payloads}
    latency = {kind: [] for kind in payloads}
    last_pub_s = {}
    max_items, max_bytes = 0, 0
    next_drain_s = slow_period_s
    kind_of_item = {}
    for kind, payload in payloads.items():
        for item in (payload if isinstance(payload, tuple) else (payload,)):
            kind_of_item[id(item)] = kind

    def drain_slow(now_s):
        nonlocal max_items, max_bytes
        items = slow.take_all()
        max_items = max(max_items, len(items))
        max_bytes = max(max_bytes, sum(len(i) for i in items))
        for item in items:
            if isinstance(item, bytes):  # map binary: counted with its map_meta
                continue
            kind = kind_of_item[id(item)]
            delivered[kind] += 1
            latency[kind].append(now_s - last_pub_s[kind])

    for when_s, kind in timeline(duration_s):
        while next_drain_s <= when_s:
            drain_slow(next_drain_s)
            next_drain_s += slow_period_s
        last_pub_s[kind] = when_s
        published[kind] += 1
        start = time.perf_counter()
        hub.publish(kind, payloads[kind])
        publish_cost[kind].append(time.perf_counter() - start)
        for channel in channels[:-1]:  # fast clients: drained at once
            channel.take_all()
    drain_slow(duration_s)
    rows = []
    for kind, samples in publish_cost.items():
        lost = published[kind] - delivered[kind]
        extra = (f'{payload_bytes(payloads[kind]) / 1024:.1f} KB/msg; slow client '
                 f'({slow_period_s:.0f} s per batch): published {published[kind]}, delivered '
                 f'{delivered[kind]}, coalesced {lost} '
                 f'({100.0 * lost / max(1, published[kind]):.0f} %), '
                 f'sim latency max {max(latency[kind], default=0.0):.2f} s')
        rows.append(Row('hub', f'hub.publish {kind}', f'{clients} clients', samples,
                        RATES[kind], in_total=True, extra=extra))
    rows[0].extra += (f'; slow-client queue: at most {max_items} items, '
                      f'{max_bytes / 1024:.1f} KB pending at a drain (one slot per kind)')
    rows.append(bench_wakeup(clients))
    return rows


def bench_wakeup(clients, calls=500):
    """Time loop.call_soon_threadsafe, the real wakeup of a client of the hub."""
    loop = asyncio.new_event_loop()
    try:
        samples = []
        for _ in range(calls):
            start = time.perf_counter()
            loop.call_soon_threadsafe(int)
            samples.append(time.perf_counter() - start)
            loop.run_until_complete(asyncio.sleep(0))
    finally:
        loop.close()
    per_cycle = [s * clients for s in samples]
    rate = RATES['pose'] + RATES['scan'] + RATES['status'] + RATES['map']
    return Row('hub', 'wakeup call_soon_threadsafe', f'x{clients} per publish', per_cycle,
               rate, in_total=True, extra='cost of the 5 wakeups of one publish')


def bench_hub_real(seconds, map_side, clients=5):
    """Run the hub with real threads and the real clock; return (rows, text summary)."""
    payloads = hub_payloads(map_side)
    hub = BroadcastHub(max_clients=clients)
    pub_time = {}
    stop = threading.Event()
    stats = [{'delivered': 0, 'lat': []} for _ in range(clients)]

    def consumer(index):
        wake = events[index]
        channel = channels[index]
        while not stop.is_set():
            wake.wait(0.05)
            wake.clear()
            items = channel.take_all()
            now = time.perf_counter()
            for item in items:
                stats[index]['delivered'] += 1
                if isinstance(item, str):
                    stats[index]['lat'].append(now - pub_time.get(item[9:13], now))
            if index == clients - 1:
                time.sleep(0.5)  # slow client

    events = [threading.Event() for _ in range(clients)]
    channels = [hub.add_client(events[i].set) for i in range(clients)]
    threads = [threading.Thread(target=consumer, args=(i,), daemon=True)
               for i in range(clients)]
    for thread in threads:
        thread.start()
    cpu_start, wall_start = time.process_time(), time.perf_counter()
    cost = {kind: [] for kind in payloads}
    origin = time.perf_counter()
    for when_s, kind in timeline(seconds):
        delay = origin + when_s - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        text = payloads[kind] if isinstance(payloads[kind], str) else payloads[kind][0]
        pub_time[text[9:13]] = time.perf_counter()
        start = time.perf_counter()
        hub.publish(kind, payloads[kind])
        cost[kind].append(time.perf_counter() - start)
    stop.set()
    for thread in threads:
        thread.join(2.0)
    cpu = time.process_time() - cpu_start
    wall = time.perf_counter() - wall_start
    rows = [Row('hub-real', f'hub.publish {k} (threads)', f'{clients} clients', v, RATES[k])
            for k, v in cost.items()]
    fast = [x for s in stats[:-1] for x in s['lat']]
    summary = (f'real-hub: {seconds:.0f} s wall, process CPU {100.0 * cpu / wall:.1f} % of '
               f'one core (publisher + {clients} consumers), delivery latency of fast '
               f'clients: mean {fmt_ms(sum(fast) / max(1, len(fast)))} ms, max '
               f'{fmt_ms(max(fast, default=0.0))} ms; slow client delivered '
               f'{stats[-1]["delivered"]} items')
    return rows, summary


# --- (e) teleop -------------------------------------------------------------------------
class FakeClock:
    """Manual clock in seconds."""

    def __init__(self):
        """Start at 100 s."""
        self.now = 100.0

    def __call__(self):
        """Return the current fake time."""
        return self.now


def bench_teleop(calls):
    """Time TeleopGate.on_teleop at 30 Hz and watchdog_tick at 50 Hz."""
    clock = FakeClock()
    gate = TeleopGate(0.2, 0.8, 0.3, clock=clock)
    on_samples = []
    for seq in range(calls):
        clock.now += 1.0 / RATES['teleop'] * 1.01  # just under 30 Hz: the gate drops > 30 Hz
        message = {'type': 'teleop', 'vx': 0.1, 'wz': 0.2, 'seq': seq}
        start = time.perf_counter()
        action = gate.on_teleop('client-1', message, False)
        on_samples.append(time.perf_counter() - start)
        assert action.kind == 'publish', action  # a wrong benchmark must fail, not mislead
    fresh_samples = []
    for _ in range(calls):
        clock.now += 1.0 / RATES['watchdog'] / 100.0  # stays inside the 0.3 s lease
        start = time.perf_counter()
        gate.watchdog_tick()
        fresh_samples.append(time.perf_counter() - start)
    clock.now += 10.0
    expired = []
    for _ in range(calls):
        gate.on_teleop('client-1', {'vx': 0.1, 'wz': 0.0, 'seq': 10 ** 9 + len(expired)},
                       False)
        clock.now += 10.0
        start = time.perf_counter()
        gate.watchdog_tick()
        expired.append(time.perf_counter() - start)
    return [Row('teleop', 'TeleopGate.on_teleop', 'accepted, 1 client', on_samples,
                RATES['teleop'], in_total=True),
            Row('teleop', 'TeleopGate.watchdog_tick', 'owner fresh', fresh_samples,
                RATES['watchdog'], in_total=True),
            Row('teleop', 'TeleopGate.watchdog_tick', 'owner expired (zero burst)', expired,
                RATES['watchdog'])]


# --- (f) map listing --------------------------------------------------------------------
def bench_listing(repeat, maps, entries):
    """Time list_maps and MapSaver.names() on a directory with `entries` entries."""
    with tempfile.TemporaryDirectory(prefix='bench_maps_') as folder:
        for index in range(maps):
            for ext in ('.yaml', '.pgm', '.posegraph', '.data'):
                with open(os.path.join(folder, f'map_{index:04d}{ext}'), 'w') as handle:
                    handle.write('x')
        for index in range(entries - 4 * maps):
            with open(os.path.join(folder, f'other_{index:05d}.txt'), 'w') as handle:
                handle.write('x')
        found = list_maps(folder)
        assert len(found) == min(maps, 200), len(found)
        saver = MapSaver(folder, None)
        rows = [Row('listing', 'list_maps', f'{maps} maps, {entries} entries',
                    time_calls(lambda: list_maps(folder), repeat), RATES['listing']),
                Row('listing', 'MapSaver.names', f'{maps} maps, {entries} entries',
                    time_calls(saver.names, repeat), RATES['listing'])]
    return rows


# --- reporting --------------------------------------------------------------------------
def render(rows):
    """Return the Markdown table of `rows` followed by the Pi 5 hypothesis summary."""
    lines = ['| Benchmark | Detail | Rate (Hz) | n | mean (ms) | p95 (ms) | max (ms) '
             '| % core here | % core Pi x3 (hyp.) | % core Pi x5 (hyp.) | > 70 % on Pi? |',
             '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|']
    flagged = []
    for row in rows:
        over = row.exceeded()
        verdict = 'no' if not over else 'YES (x' + ', x'.join(f'{s:g}' for s in over) + ')'
        if over:
            flagged.append(f'{row.name} [{row.detail}]')
        lines.append(
            f'| {row.name} | {row.detail} | {row.rate_hz:g} | {row.count} | {fmt_ms(row.mean)} '
            f'| {fmt_ms(row.p95)} | {fmt_ms(row.maximum)} | {100 * row.core_share():.2f} '
            f'| {100 * row.core_share(3.0):.1f} | {100 * row.core_share(5.0):.1f} | {verdict} |')
    lines.append('')
    lines.append('Notes per row:')
    lines.extend(f'- {r.name} [{r.detail}]: {r.extra}' for r in rows if r.extra)
    totals = [r for r in rows if r.in_total]
    if totals:
        here = sum(r.core_share() for r in totals)
        lines.append('')
        lines.append(
            f'Reference load (rows marked as total: scan 720 beams, map 1000x1000 array, '
            f'plan, hub, teleop; all in Python threads sharing one GIL): {100 * here:.1f} % '
            f'of one core here, {100 * here * 3:.1f} % (x3) / {100 * here * 5:.1f} % (x5) '
            f'on the Pi (hypothesis).')
    lines.append('')
    lines.append('Operations above 70 % of one core on a Pi 5 (hypothesis x3-x5): '
                 + ('; '.join(flagged) if flagged else 'none') + '.')
    return '\n'.join(lines)


def environment():
    """Return a one-line description of the machine and interpreter."""
    return (f'{platform.platform()}; {platform.processor() or platform.machine()}; '
            f'Python {platform.python_version()}; CPUs {os.cpu_count()}')


def run_groups(args):
    """Run the selected groups and return their rows (and the real-hub summary)."""
    quick = args.quick
    repeat = args.repeat or (3 if quick else 10)
    sides = (500, 1000) if quick else MAP_SIDES
    selected = set(args.only.split(',')) if args.only else set(GROUPS)
    unknown = selected - set(GROUPS)
    if unknown:
        raise SystemExit(f'Unknown group(s): {sorted(unknown)}; valid: {GROUPS}')
    rows, summary = [], ''
    if 'maps' in selected:
        rows += bench_maps(repeat, sides)
    if 'scan' in selected:
        rows += bench_scan(repeat * (5 if quick else 20))
    if 'path' in selected:
        rows += bench_path(repeat * (5 if quick else 20))
    if 'hub' in selected:
        rows += bench_hub(3.0 if quick else 10.0, 500 if quick else 1000)
    if args.real_hub:
        real_rows, summary = bench_hub_real(args.real_hub, 500 if quick else 1000)
        rows += real_rows
    if 'teleop' in selected:
        rows += bench_teleop(300 if quick else 3000)
    if 'listing' in selected:
        maps, entries = (20, 1000) if quick else (200, 10000)
        rows += bench_listing(repeat, maps, entries)
    return rows, summary


def parse_args(argv):
    """Parse the command line."""
    parser = argparse.ArgumentParser(description='Benchmark of the web bridge components.')
    parser.add_argument('--quick', action='store_true', help='short run (about 1-2 s)')
    parser.add_argument('--repeat', type=int, default=0, help='repetitions (default 10)')
    parser.add_argument('--only', default='', help=f'comma separated groups: {GROUPS}')
    parser.add_argument('--profile', action='store_true', help='print a cProfile summary')
    parser.add_argument('--real-hub', type=float, default=0.0, metavar='SECONDS',
                        help='also run the hub with real threads and clock')
    return parser.parse_args(argv)


def main(argv=None):
    """Run the benchmark, print the Markdown report and return the process exit code."""
    args = parse_args(sys.argv[1:] if argv is None else argv)
    profiler = cProfile.Profile() if args.profile else None
    start = time.perf_counter()
    if profiler:
        profiler.enable()
    rows, summary = run_groups(args)
    if profiler:
        profiler.disable()
    print(f'### Web bridge benchmark ({"quick" if args.quick else "full"})')
    print(f'Machine: {environment()}')
    print()
    print(render(rows))
    if summary:
        print()
        print(summary)
    print()
    print(f'Peak RSS of the process: {peak_rss_mb():.0f} MB; '
          f'benchmark wall time: {time.perf_counter() - start:.1f} s')
    if profiler:
        buffer = io.StringIO()
        pstats.Stats(profiler, stream=buffer).sort_stats('cumulative').print_stats(15)
        print()
        print(buffer.getvalue())
    return 0


if __name__ == '__main__':
    sys.exit(main())
