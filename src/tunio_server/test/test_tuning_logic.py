"""tuning_logic 순수 함수 테스트 (로봇/마이크 없이 실행 가능)."""
import pytest

from tunio_interfaces.msg import StringResult, TuningState

from tunio_server import tuning_logic as L
from tunio_server.peg_calibration import StringCalibration

S = StringResult.RESULT_SUCCESS
F = StringResult.RESULT_FAILED
P = StringResult.RESULT_PENDING

# 테스트용 가짜 캘리브레이션. 실측값이 아니라 계산이 맞는지만 보기 위한 숫자다.
# 실제 값은 tools/peg_cal.py 로 재서 config/peg_calibration.yaml 에 넣는다.
CAL = StringCalibration(
    cents_per_deg=5.0,          # 1도당 5센트 -> 10센트는 2도
    min_effective_deg=0.5,
    max_angle_deg=20.0,
    backlash_release_cents=25.0,
)


def test_error_cents_zero():
    assert abs(L.error_cents(440.0, 440.0)) < 1e-9


def test_in_tune():
    assert L.is_in_tune(329.6, 329.6276, 5.0)
    assert not L.is_in_tune(340.0, 329.6276, 5.0)


def test_peg_direction():
    direction, _ = L.peg_angle(-10.0, CAL)   # 음이 낮음 -> 높여야(조임)
    assert direction == 1
    direction, _ = L.peg_angle(10.0, CAL)    # 음이 높음 -> 풀어야
    assert direction == -1


def test_peg_angle_is_positive_magnitude():
    for err in (-30.0, -1.0, 1.0, 30.0):
        _, angle = L.peg_angle(err, CAL)
        assert angle > 0


def test_peg_angle_uses_calibration_only():
    """회전량은 cents_per_deg 로만 나온다. 코드에 계수가 남아 있으면 이 값이 안 맞는다."""
    _, angle = L.peg_angle(-10.0, CAL)
    assert angle == pytest.approx(2.0)


def test_peg_angle_respects_min_effective():
    _, angle = L.peg_angle(-0.5, CAL)        # 0.1도가 나오지만 페그가 안 움직인다
    assert angle == pytest.approx(CAL.min_effective_deg)


def test_peg_angle_respects_max():
    _, angle = L.peg_angle(-1000.0, CAL)
    assert angle == pytest.approx(CAL.max_angle_deg)


def test_min_step_cents():
    assert L.min_step_cents(CAL) == pytest.approx(2.5)


def test_adjust_plan_flat_string_tightens_once():
    """음이 낮으면 이미 조이는 방향이라 goal 1개다."""
    plan = L.adjust_plan(-10.0, CAL)
    assert len(plan) == 1
    assert plan[0][0] == 1


def test_adjust_plan_sharp_string_releases_then_tightens():
    """음이 높으면 목표 아래로 풀었다가 조여 올린다 (CLAUDE.md 3.2)."""
    plan = L.adjust_plan(10.0, CAL)
    assert len(plan) == 2
    assert plan[0][0] == -1          # 먼저 푼다
    assert plan[-1][0] == 1          # 마지막은 반드시 조이는 방향
    # 푸는 양 = 오차 + 백래시 여유 = 35센트 = 7도
    assert plan[0][1] == pytest.approx(7.0)
    # 조여 올리는 양 = 백래시 여유 25센트 = 5도
    assert plan[1][1] == pytest.approx(5.0)


def test_adjust_plan_clamped_release_skips_tighten():
    """푸는 각도가 상한에 걸리면 그 회차는 풀기만 하고 끝낸다."""
    plan = L.adjust_plan(500.0, CAL)
    assert len(plan) == 1
    assert plan[0][0] == -1
    assert plan[0][1] == pytest.approx(CAL.max_angle_deg)


def test_overall():
    assert L.overall_result([S, S, S, S]) == TuningState.OVERALL_SUCCESS
    assert L.overall_result([S, F, S, S]) == TuningState.OVERALL_PARTIAL
    assert L.overall_result([F, F, F, F]) == TuningState.OVERALL_FAILED


def test_overall_pending():
    assert L.overall_result([]) == TuningState.OVERALL_PENDING
    assert L.overall_result([S, P, S, S]) == TuningState.OVERALL_PENDING


# ---------------------------------------------------------------- 작업 중 자동 보정

def test_effective_cal_scales_only_cents_per_deg():
    eff = L.effective_cal(CAL, 0.8)
    assert eff.cents_per_deg == pytest.approx(4.0)
    assert (eff.min_effective_deg, eff.max_angle_deg, eff.backlash_release_cents) == (
        CAL.min_effective_deg, CAL.max_angle_deg, CAL.backlash_release_cents)


def test_lower_gain_turns_further_for_same_error():
    """효율이 낮게 배워지면 같은 오차에 더 많이 돌린다 — 덜 조여지던 문제를 메운다."""
    (_, base), = L.adjust_plan(-10.0, CAL)
    (_, more), = L.adjust_plan(-10.0, L.effective_cal(CAL, 0.8))
    assert more == pytest.approx(base / 0.8)


def test_learn_gain_moves_toward_observed_efficiency():
    # 2도 조여 10센트를 기대했는데 7.5센트 올랐다 -> 관측 0.75, 무게 0.5 -> 0.875
    gain, note = L.learn_gain(1.0, CAL, [(1, 2.0)], -12.0, -4.5)
    assert gain == pytest.approx(1.0 - L.GAIN_ALPHA * 0.25)
    assert '75%' in note


def test_learn_gain_is_clamped():
    gain, _ = L.learn_gain(1.0, CAL, [(1, 2.0)], -30.0, 0.0)     # 관측 3.0배
    assert gain == pytest.approx(L.GAIN_MAX)
    gain, _ = L.learn_gain(L.GAIN_MIN, CAL, [(1, 10.0)], -60.0, -56.0)   # 관측 0.08배
    assert gain == pytest.approx(L.GAIN_MIN)


@pytest.mark.parametrize('plan,before,after', [
    ([(-1, 7.0), (1, 5.0)], 10.0, 2.0),     # 풀기→조임: 백래시가 섞여 배우지 않는다
    ([(1, 2.0)], -12.0, -10.5),             # 변화 1.5센트: 측정 흔들림과 못 가른다
    ([(1, 2.0)], -12.0, -14.0),             # 거꾸로 움직임: 이상치로 보고 건너뛴다
])
def test_learn_gain_skips_unreliable_cases(plan, before, after):
    gain, note = L.learn_gain(0.9, CAL, plan, before, after)
    assert gain == 0.9 and note.startswith('건너뜀')
