"""조율 루프가 조절 결과로 페그 효율을 배워 다음 조절에 쓰고, 기록을 남기는지 본다.

가짜 페그는 yaml 기울기의 75% 만 음을 바꾼다 — 2번 줄 실측(71%)과 비슷한 상황이다.
"""
import json
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tunio_server import task_manager as TM
from tunio_server import tuning_logic as L
from tunio_server.peg_calibration import StringCalibration

CAL = StringCalibration(cents_per_deg=1.065, min_effective_deg=4.7,
                        max_angle_deg=60.0, backlash_release_cents=30.0)
TARGET = L.TARGETS[2][1]


def hz(cents):
    return TARGET * 2 ** (cents / 1200.0)


def make_node(tmp_path, start_cents, efficiency, tolerance=2.0):
    state = {'cents': start_cents, 'angles': []}

    def measure(no, budget):
        return TM.Measurement(True, hz(state['cents'])), 0

    def run_action(client, goal, timeout, what):
        state['angles'].append(goal.direction * goal.angle_deg)
        state['cents'] += goal.direction * goal.angle_deg * CAL.cents_per_deg * efficiency
        return SimpleNamespace(success=True, message='')

    n = SimpleNamespace(
        results={2: SimpleNamespace(target_hz=TARGET)},
        active_settings=SimpleNamespace(max_measure_retry=3, max_adjust_count=5,
                                        tolerance_cents=tolerance),
        calibration=SimpleNamespace(get=lambda no: CAL),
        _gain={}, _adjust_log=[], job_id='gain-test', adjust_count=0,
        cli_adjust=object(), _run_action=run_action, _measure_with_retry=measure,
        _p=lambda key: {'adjust_log_dir': str(tmp_path), 'adjust_timeout_sec': 1.0}[key],
        _gate=lambda: None, _publish_state=lambda: None, _set_phase=lambda *a: None,
        get_logger=lambda: Mock())
    for name in ('_adjust_peg', '_learn_gain', '_write_adjust_log'):
        setattr(n, name, getattr(TM.TaskManager, name).__get__(n))
    return n, state


def test_second_adjust_turns_further_after_learning(tmp_path):
    n, state = make_node(tmp_path, start_cents=-60.0, efficiency=0.75)
    TM.TaskManager._tune_string(n, 2)
    first, second = state['angles'][:2]
    assert first == pytest.approx(60.0 / CAL.cents_per_deg)
    # 첫 조절에서 75% 를 보고 gain 0.875 를 배웠다 -> 두 번째는 같은 오차에 1/0.875 배 더 돌린다
    remaining = 60.0 - first * CAL.cents_per_deg * 0.75
    assert second == pytest.approx(remaining / (CAL.cents_per_deg * 0.875))
    assert n._gain[2] < 1.0


def test_learning_reaches_tolerance_in_fewer_adjusts(tmp_path):
    """같은 조건에서 보정이 없을 때보다 조절 횟수가 줄어든다."""
    n, state = make_node(tmp_path, start_cents=-60.0, efficiency=0.75)
    TM.TaskManager._tune_string(n, 2)
    learned = len(state['angles'])

    # 보정 없이: 매번 75% 씩만 줄어든다 -> 60 * 0.25^k <= 2
    fixed = math.ceil(math.log(2.0 / 60.0) / math.log(0.25))
    assert learned <= fixed


def test_adjust_log_records_prediction_vs_observation(tmp_path):
    n, _ = make_node(tmp_path, start_cents=-30.0, efficiency=0.75, tolerance=10.0)
    TM.TaskManager._tune_string(n, 2)
    doc = json.loads((tmp_path / 'gain-test_peg_adjust.json').read_text(encoding='utf-8'))
    rec = doc['records'][0]
    assert rec['string_number'] == 2 and rec['gain_used'] == 1.0
    assert rec['predicted_change_cents'] == pytest.approx(30.0, abs=0.01)
    assert rec['observed_change_cents'] == pytest.approx(22.5, abs=0.01)
    assert rec['gain_after'] == pytest.approx(0.875)
