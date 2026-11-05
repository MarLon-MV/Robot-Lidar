# Benchmarks of the web bridge (task 22, criteria A-5 and A-6)

Scope: CPU/memory cost of the pure components of `mtrb_web_bridge` measured on a
development machine WITHOUT ROS, plus the exact procedure to repeat the measurement on the
Raspberry Pi 5 (A-5). Nothing in this file is a measurement of the Pi: the Pi columns are an
explicit hypothesis and the Pi tables are templates to fill in.

## 1. Method (this machine)

- Script: `test/bench_web.py` (no `test_` prefix: not collected by pytest/`colcon test`).
  Run from `Proyecto/src/mtrb_bringup`:
  `../../.venv/bin/python -I test/bench_web.py [--quick] [--only maps,scan,path,hub,teleop,listing]
  [--repeat N] [--profile] [--real-hub SECONDS]`.
- Time per call with `time.perf_counter`; peak Python memory of one run with `tracemalloc`;
  peak RSS with `resource`. Each case is repeated N times (10 by default; fewer for maps
  >= 2000x2000) and mean / p95 (nearest rank) / max are reported.
- "% core here" = mean time per cycle x real rate (Hz) x 100. The Pi columns multiply it by
  3 and 5. HYPOTHESIS (not measured): a Pi 5 is 3-5 times slower than this machine for this
  pure-Python work. A row is flagged when it alone exceeds 70 % of one core.
- Maps: pseudo-realistic grids (unknown margin, walls with door gaps, free interior,
  isolated noisy cells), as `array('b')` (what rclpy delivers) and as `list`.
  Pipeline = blake2b change hash (only possible on a buffer) + `occupancy_to_bytes` (zlib
  level 1) + `pack_map_binary`, at the 1 Hz maximum of the bridge. The `list` rows time
  `occupancy_to_bytes` alone, which converts a `list` with `array.array('b', data)` (C
  speed) before compressing; the node does that same conversion once in `_broadcast_map`
  and then hashes the array, so a real `list` costs about the `list` row plus the hash of
  the `array(b)` row (the hash is not added in the `list` rows).
- Scan: `downsample_scan` + `scan_to_map_points` + JSON for 360 / 720 / 1800 beams at 5 Hz,
  `scan.max_points` = 360.
- Hub: `BroadcastHub` with 5 clients (maximum), pose 10 Hz, scan 5 Hz, status 1 Hz, map 1 Hz,
  10 s of SIMULATED time (deterministic); 4 fast clients drain at once and 1 slow client
  drains one batch every 2 s. `(threads)` rows come from `--real-hub 5` (real threads and
  clock, slow client sleeping 0.5 s per batch).
- Teleop: `TeleopGate.on_teleop` at 30 Hz and `watchdog_tick` at 50 Hz (fake clock).
- Listing: `list_maps` and `MapSaver.names` with 200 maps and 10000 directory entries
  (`MAP_SCAN_MAX_ENTRIES`), in a temporary directory that is removed afterwards.
- Not covered here (needs ROS/Pi): rclpy (de)serialization of `/map` and `/scan`, DDS, TF
  lookups, uvicorn/starlette `send` per client, the browser. They only show up in the Pi
  procedure of section 3.

## 2. Results on this machine

- CPU: Apple M2 (8 cores), macOS 27.0.1 (Darwin 27.0.0, arm64), Python 3.12.6.
- Full run (`--real-hub 5`), 9.5 s of wall time, peak RSS 267 MB (dominated by the
  4000x4000 `list`: 16 M Python references, about 128 MB). Figures re-measured after the
  `list` -> `array('b')` conversion was added to the node (see Reading).

| Benchmark | Detail | Rate (Hz) | n | mean (ms) | p95 (ms) | max (ms) | % core here | % core Pi x3 (hyp.) | % core Pi x5 (hyp.) | > 70 % on Pi? |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| map 500x500 | array(b) | 1 | 10 | 0.5366 | 0.5969 | 0.5969 | 0.05 | 0.2 | 0.3 | no |
| map 500x500 | list | 1 | 10 | 3.43 | 3.49 | 3.49 | 0.34 | 1.0 | 1.7 | no |
| map 1000x1000 | array(b) | 1 | 10 | 2.13 | 2.34 | 2.34 | 0.21 | 0.6 | 1.1 | no |
| map 1000x1000 | list | 1 | 10 | 13.54 | 13.95 | 13.95 | 1.35 | 4.1 | 6.8 | no |
| map 2000x2000 | array(b) | 1 | 5 | 8.73 | 9.21 | 9.21 | 0.87 | 2.6 | 4.4 | no |
| map 2000x2000 | list | 1 | 5 | 55.10 | 55.87 | 55.87 | 5.51 | 16.5 | 27.5 | no |
| map 4000x4000 | array(b) | 1 | 5 | 37.93 | 50.21 | 50.21 | 3.79 | 11.4 | 19.0 | no |
| map 4000x4000 | list | 1 | 5 | 220 | 225 | 225 | 22.02 | 66.1 | 110.1 | YES (x5) |
| scan 360 beams | array(f) | 5 | 200 | 0.4345 | 0.4498 | 0.6272 | 0.22 | 0.7 | 1.1 | no |
| scan 360 beams | list | 5 | 200 | 0.4295 | 0.4403 | 0.5072 | 0.21 | 0.6 | 1.1 | no |
| scan 720 beams | array(f) | 5 | 200 | 0.5526 | 0.5895 | 0.7863 | 0.28 | 0.8 | 1.4 | no |
| scan 720 beams | list | 5 | 200 | 0.5432 | 0.5650 | 0.5898 | 0.27 | 0.8 | 1.4 | no |
| scan 1800 beams | array(f) | 5 | 200 | 0.8235 | 0.8724 | 1.29 | 0.41 | 1.2 | 2.1 | no |
| scan 1800 beams | list | 5 | 200 | 0.7919 | 0.8203 | 0.9375 | 0.40 | 1.2 | 2.0 | no |
| simplify_path | 2000 -> 200 points | 1 | 200 | 0.2264 | 0.1753 | 7.08 | 0.02 | 0.1 | 0.1 | no |
| hub.publish pose | 5 clients | 10 | 100 | 0.0014 | 0.0014 | 0.0039 | 0.00 | 0.0 | 0.0 | no |
| hub.publish scan | 5 clients | 5 | 50 | 0.0014 | 0.0015 | 0.0022 | 0.00 | 0.0 | 0.0 | no |
| hub.publish status | 5 clients | 1 | 10 | 0.0014 | 0.0015 | 0.0015 | 0.00 | 0.0 | 0.0 | no |
| hub.publish map | 5 clients | 1 | 10 | 0.0013 | 0.0013 | 0.0013 | 0.00 | 0.0 | 0.0 | no |
| wakeup call_soon_threadsafe | x5 per publish | 17 | 500 | 0.0060 | 0.0073 | 0.0556 | 0.01 | 0.0 | 0.1 | no |
| hub.publish pose (threads) | 5 clients | 10 | 50 | 0.0768 | 0.1388 | 0.1497 | 0.08 | 0.2 | 0.4 | no |
| hub.publish scan (threads) | 5 clients | 5 | 25 | 0.0796 | 0.1099 | 0.1222 | 0.04 | 0.1 | 0.2 | no |
| hub.publish status (threads) | 5 clients | 1 | 5 | 0.0172 | 0.0259 | 0.0259 | 0.00 | 0.0 | 0.0 | no |
| hub.publish map (threads) | 5 clients | 1 | 5 | 0.0876 | 0.1686 | 0.1686 | 0.01 | 0.0 | 0.0 | no |
| TeleopGate.on_teleop | accepted, 1 client | 30 | 3000 | 0.0058 | 0.0073 | 0.1275 | 0.02 | 0.1 | 0.1 | no |
| TeleopGate.watchdog_tick | owner fresh | 50 | 3000 | 0.0007 | 0.0008 | 0.0222 | 0.00 | 0.0 | 0.0 | no |
| TeleopGate.watchdog_tick | owner expired (zero burst) | 50 | 3000 | 0.0007 | 0.0008 | 0.0021 | 0.00 | 0.0 | 0.0 | no |
| list_maps | 200 maps, 10000 entries | 1 | 10 | 9.28 | 9.42 | 9.42 | 0.93 | 2.8 | 4.6 | no |
| MapSaver.names | 200 maps, 10000 entries | 1 | 10 | 9.69 | 10.68 | 10.68 | 0.97 | 2.9 | 4.8 | no |

Notes per row:
- map 500x500 [array(b)]: hash 0.2398 ms; encode 0.2952 ms; zlib 7.7 KB (32:1); peak mem 1 MB
- map 500x500 [list]: hash n/a; encode 3.43 ms; zlib 7.7 KB (32:1); peak mem 1 MB
- map 1000x1000 [array(b)]: hash 0.9604 ms; encode 1.16 ms; zlib 19.7 KB (50:1); peak mem 1 MB
- map 1000x1000 [list]: hash n/a; encode 13.54 ms; zlib 19.7 KB (50:1); peak mem 2 MB
- map 2000x2000 [array(b)]: hash 4.04 ms; encode 4.69 ms; zlib 58.4 KB (67:1); peak mem 4 MB
- map 2000x2000 [list]: hash n/a; encode 55.09 ms; zlib 58.4 KB (67:1); peak mem 8 MB
- map 4000x4000 [array(b)]: hash 17.17 ms; encode 20.75 ms; zlib 158.9 KB (98:1); peak mem 16 MB
- map 4000x4000 [list]: hash n/a; encode 220 ms; zlib 158.9 KB (98:1); peak mem 31 MB
- scan 360 beams [array(f)]: downsample 0.0851 ms; project 0.2364 ms; json 0.1130 ms; message 4.5 KB
- scan 360 beams [list]: downsample 0.0816 ms; project 0.2367 ms; json 0.1112 ms; message 4.5 KB
- scan 720 beams [array(f)]: downsample 0.1850 ms; project 0.2494 ms; json 0.1183 ms; message 4.7 KB
- scan 720 beams [list]: downsample 0.1766 ms; project 0.2486 ms; json 0.1180 ms; message 4.7 KB
- scan 1800 beams [array(f)]: downsample 0.4518 ms; project 0.2512 ms; json 0.1205 ms; message 4.7 KB
- scan 1800 beams [list]: downsample 0.4254 ms; project 0.2491 ms; json 0.1173 ms; message 4.7 KB
- hub.publish pose [5 clients]: 0.1 KB/msg; slow client (2 s per batch): published 100, delivered 5, coalesced 95 (95 %), sim latency max 0.10 s; slow-client queue: at most 5 items, 24.6 KB pending at a drain (one slot per kind)
- hub.publish scan [5 clients]: 4.4 KB/msg; slow client (2 s per batch): published 50, delivered 5, coalesced 45 (90 %), sim latency max 0.15 s
- hub.publish status [5 clients]: 0.3 KB/msg; slow client (2 s per batch): published 10, delivered 5, coalesced 5 (50 %), sim latency max 0.70 s
- hub.publish map [5 clients]: 19.8 KB/msg; slow client (2 s per batch): published 10, delivered 5, coalesced 5 (50 %), sim latency max 0.50 s
- wakeup call_soon_threadsafe [x5 per publish]: cost of the 5 wakeups of one publish

Reference load (rows marked as total: scan 720 beams, map 1000x1000 array, plan, hub, teleop; all in Python threads sharing one GIL): 0.5 % of one core here, 1.6 % (x3) / 2.7 % (x5) on the Pi (hypothesis).

Operations above 70 % of one core on a Pi 5 (hypothesis x3-x5): map 4000x4000 [list].

real-hub: 5 s wall, process CPU 0.6 % of one core (publisher + 5 consumers), delivery latency of fast clients: mean 0.2254 ms, max 2.90 ms; slow client delivered 34 items

Reading:

- Every operation with the production data type (`array('b')` / `array('f')`) is far below
  70 % of a core, even with the x5 hypothesis. The reference load (scan 720, map 1000x1000,
  plan, hub, teleop) is 0.6 % of one core here (about 3 % at x5).
- The only row above 70 % on a Pi (hypothesis, x5) is the 4000x4000 map delivered as a
  Python `list`. `_broadcast_map` and `occupancy_to_bytes` no longer fail or loop per cell
  with a `list`: they convert it once to `array('b')` in C (about 14 ms per million cells,
  linear; the old per-cell Python loop took about 50 ms per million). With `array('b')`
  the buffer fast path is used (about 38 ms for 4000x4000, hash included in the row notes).
  rclpy gives `array.array('b')` for `int8[]`, so the `list` path is defensive and should
  not happen on the robot; it is only measured here.
- A 4000x4000 `array('b')` map costs about 38 ms per cycle (hash 17 + encode 21), peak 16 MB
  and a 159 KB payload; the 4000x4000 map is below the CPU limit but the rclpy
  deserialization of 16 MB per `/map` message (not measured here) is the real risk.
- Hub: a `publish` with 5 clients costs about 1.5 microseconds (plus about 13 microseconds of
  wakeups); the slow client coalesces 90-95 % of pose/scan messages and keeps at most
  one pending item per kind (5 items, 25 KB with the 1000x1000 map), without slowing
  the fast clients (delivery latency in `--real-hub`: mean 0.26 ms, max 3.3 ms).
- Note: the node hashes a buffer (`memoryview(data)`). `_broadcast_map` converts anything
  that is not an `array`/`memoryview`/`bytes`/`bytearray` to `array('b')` first, so a plain
  `list` no longer raises `TypeError`; data that cannot be converted is reported as an
  invalid map without echoing cell values.

## 3. Procedure for the Pi 5 (A-5), TEMPLATE to fill in

Goal (A-5 of the spec): with the Pi above 70 % CPU during mapping, the web keeps responding
without visible loss of fluidity on a phone. Reference limits: pose latency < 300 ms,
map latency < 2 s.

### 3.1 Preparation

1. On the Pi: give the node a token of at least 20 characters (not a ROS parameter, and no
   `web.local.yaml`): either `export MTRB_WEB_TOKEN="$(openssl rand -base64 24)"` or create
   `~/.config/mtrb/web_token` (regular file, owned by the user, mode 0600:
   `install -m 600 /dev/null ~/.config/mtrb/web_token`, then write the token into it; the
   environment variable wins if both exist). The node refuses to start without a token,
   with a placeholder or with a shorter one. Then start
   `ros2 launch mtrb_bringup mtrb.launch.py mode:=mapping web:=true` (port 8080 by default).
2. Drive the robot (or replay a rosbag) so that SLAM, Nav2 and the LiDAR are really working.
   Let it run 2 minutes before measuring.
3. Note the Pi OS, `python3 --version`, `cat /proc/cpuinfo | grep Model` and
   `vcgencmd measure_clock arm` (check no thermal throttling: `vcgencmd get_throttled`
   must be `0x0`).

### 3.2 Measurements (each for 60 s)

```bash
# Whole machine and per thread of the bridge (find the busiest thread: uvicorn, executor, ...)
top -b -n 1 | head -15
top -H -p $(pgrep -f mtrb_web_bridge)          # interactive; press 1 for per-core view
pidstat -t -p $(pgrep -f mtrb_web_bridge) 5 12  # if sysstat is installed: CPU per thread
# Input rates and bandwidth (namespace /r2)
ros2 topic hz /r2/scan
ros2 topic bw /r2/map
ros2 topic hz /r2/map
# Web traffic of the bridge (optional)
sudo iftop -i wlan0 -f "port 8080"
```

Record the CPU % of the bridge process (sum of its threads) and of the whole Pi (idle %
from `top`). Run it twice: once with 0 clients and once with 5 clients (3.3).

### 3.3 Five WebSocket clients

Browser: open `http://<ip-pi>:8080` in 5 tabs/devices (`limits.max_clients` = 5; a 6th
WebSocket is accepted and then closed with code 1013; `limits.max_connections` = 64 is the
separate cap on concurrent HTTP + WebSocket connections). The token is the same one the
node uses (at least 20 characters). Script (from a laptop on the same LAN,
`pip install websockets`; pass the real token as the second argument):

```python
import asyncio, json, sys, time, urllib.parse, websockets
URL = 'ws://{ip}:8080/ws?token={token}'.format(
    ip=sys.argv[1], token=urllib.parse.quote(sys.argv[2], safe=''))  # base64 has + / =
async def client(n, stats):
    async with websockets.connect(URL, max_size=None) as ws:
        last = {}
        end = time.time() + 60
        while time.time() < end:
            m = await asyncio.wait_for(ws.recv(), 5)
            now = time.time()
            kind = 'map' if isinstance(m, bytes) else json.loads(m)['type']
            if kind == 'pose':
                age = now - json.loads(m)['stamp']  # clocks must be synced (chrony/ntp)
                stats.setdefault((n, 'pose_age'), []).append(age)
            if kind in last:
                stats.setdefault((n, kind + '_dt'), []).append(now - last[kind])
            last[kind] = now
async def sixth():
    # The 6th client must be accepted and then closed with 1013 (try again later).
    await asyncio.sleep(1)           # the 5 others are connected by now
    async with websockets.connect(URL, max_size=None) as ws:
        try:
            await asyncio.wait_for(ws.recv(), 5)
        except websockets.exceptions.ConnectionClosed as closed:
            print('6th client closed with', closed.rcvd.code if closed.rcvd else None,
                  '(expected 1013)')
async def main():
    stats = {}
    await asyncio.gather(*(client(i, stats) for i in range(5)), sixth())
    for k, v in sorted(stats.items()):
        v.sort()
        print(k, 'n=%d mean=%.3f p95=%.3f max=%.3f' % (len(v), sum(v)/len(v),
              v[int(0.95*len(v))-1], v[-1]))
asyncio.run(main())
```

To emulate a slow client, add `await asyncio.sleep(1.5)` after each `recv()` in one of the
clients: the others must not be affected.

### 3.4 Latency in the browser

- Pose: in the web UI, move the robot (joystick or by hand) and film the screen and the
  robot together (60 fps phone video); the delay between the movement and the marker is the
  pose latency (limit 300 ms). Alternatively, in DevTools > Network > WS compare the `stamp`
  of the `pose` frames with `Date.now()/1000` (clocks synced).
- Map: drive into an unexplored area and measure (video or stopwatch) the time until the new
  area is drawn (limit 2 s; includes `map_update_interval` of slam_toolbox and the 1 Hz of
  the bridge, so it cannot be below those intervals).
- Fluidity: check by eye on a phone that the pose marker does not freeze (no gaps > 0.3 s)
  while the Pi is above 70 % CPU.

### 3.5 Pass/fail criteria for A-5

PASS only if ALL hold (with 5 clients, mapping, Pi at > 70 % CPU overall):

| Metric | Threshold | Result (Pi) |
|---|---|---|
| CPU of the whole Pi during the test | > 70 % reached (condition of the test), no throttling | |
| CPU of `mtrb_web_bridge` (sum of threads) | informative; < 35 % of one core suggested | |
| `ros2 topic hz /r2/scan` | stays at the LiDAR rate, no drops caused by the bridge | |
| Pose latency in the browser | < 300 ms (p95) | |
| Map latency in the browser | < 2 s | |
| Pose inter-arrival at the 5 clients | p95 < 0.3 s, no gap > 0.5 s | |
| Slow client | only its own messages coalesce; fast clients unaffected | |
| Web remains responsive (REST `/api/state` < 500 ms) | yes | |
| Navigation/SLAM not degraded by the web (compare with `web:=false`) | no | |

Repeat with `web:=false` to separate the cost of the web from the rest.

### 3.6 Tuning if it fails (in this order)

1. Lower the scan rate to 2 Hz (`rates.scan: 2.0` in `param/web.yaml`), as foreseen in the
   spec risks; then `rates.pose` to 5 Hz and `rates.status` to 0.5 Hz if still needed.
2. Raise `map_update_interval` in `slam_toolbox.yaml` (for example 2 s -> 5 s): fewer `/map`
   messages to deserialize, hash and compress.
3. Lower `scan.max_points` (360 -> 180) and `plan.max_points` (200 -> 100).
4. The zlib level is already 1 (`ZLIB_LEVEL`): do not go higher; going to 0 only helps if
   the bridge CPU is dominated by compression (it is not on this machine).
5. Limit the map size (slam_toolbox `resolution` 0.05 -> 0.1 gives 4x fewer cells) and
   keep the map within the explored area; maps >= 2000x2000 only if the CPU is OK.
6. Reduce `limits.max_clients` (5 -> 3) in the worst case.
7. After each change, repeat 3.2-3.5 and record it in section 4.

## 4. Pi 5 results (fill in; none have been measured yet)

| Field | Value |
|---|---|
| Date / commit / Pi OS | |
| Mode and map size reached | |
| Bridge CPU, 0 clients (`top -H`) | |
| Bridge CPU, 5 clients | |
| Whole Pi CPU | |
| `ros2 topic hz /r2/scan` | |
| `ros2 topic bw /r2/map` | |
| Pose latency / map latency | |
| Verdict A-5 (pass / fail) and tuning applied | |

## 5. Regression of the final suite (task 22, criterion A-6)

Run on this machine (Python 3.12.6, venv `Proyecto/.venv`), results in the task report:

- `../../.venv/bin/python -m pytest -q test/ --ignore=test/test_flake8.py --ignore=test/test_pep257.py`
- `../../.venv/bin/python -m flake8 .`
- pydocstyle with `--ignore=D100,D101,D102,D103,D104,D105,D106,D107,D203,D212,D404`
- `node --check web/app.js`
- bridge/map saver/params/server tests with the ROS stubs on `PYTHONPATH`.

The ROS-dependent tests (`test_initial_pose`, `test_launch`, `test_web_bridge`) are skipped
here without ROS: A-6 (`colcon build`/`colcon test`) must still be run on the Pi or in a
ROS 2 Jazzy container.
