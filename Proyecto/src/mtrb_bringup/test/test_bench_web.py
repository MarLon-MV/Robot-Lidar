"""Smoke test of test/bench_web.py: it must import without ROS and run fast in --quick."""

import importlib.util
import os
import time

import pytest

pytest.importorskip('fastapi')

BENCH_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bench_web.py')


def _load():
    spec = importlib.util.spec_from_file_location('bench_web_under_test', BENCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_quick_mode_runs_under_five_seconds_and_prints_a_table(capsys):
    bench = _load()
    start = time.perf_counter()
    code = bench.main(['--quick'])
    elapsed = time.perf_counter() - start
    out = capsys.readouterr().out
    assert code == 0
    assert elapsed < 5.0
    assert '| Benchmark |' in out
    for name in ('map 500x500', 'scan 720 beams', 'simplify_path', 'hub.publish scan',
                 'TeleopGate.on_teleop', 'list_maps'):
        assert name in out


def test_percentile_is_nearest_rank():
    bench = _load()
    assert bench.percentile([5, 1, 3, 2, 4], 0.95) == 5
    assert bench.percentile([1, 2, 3, 4, 5], 0.5) == 3


def test_row_flags_operation_over_seventy_percent_on_slower_pi():
    bench = _load()
    row = bench.Row('g', 'n', 'd', [0.05], 5.0)  # 25 % here -> 75 % at x3, 125 % at x5
    assert row.exceeded() == [3.0, 5.0]
    assert bench.Row('g', 'n', 'd', [0.001], 5.0).exceeded() == []


def test_unknown_group_is_rejected():
    bench = _load()
    with pytest.raises(SystemExit):
        bench.main(['--only', 'nonexistent'])
