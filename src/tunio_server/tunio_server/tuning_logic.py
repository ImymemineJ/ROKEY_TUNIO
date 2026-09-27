"""
조율 판단 로직 (rclpy 비의존 순수 함수 모음).

task_manager가 이 함수들을 호출한다. 단위 테스트가 쉽도록 분리.

열거값은 정의를 중복하지 않고 tunio_interfaces 메시지에서 직접 가져온다.
메시지 모듈 import만 하므로 rclpy 초기화나 ROS 그래프 없이도 동작한다.
"""
import dataclasses
import math

from tunio_interfaces.msg import StringResult, TuningState

# 목표 주파수 (A4=440, 12평균율). 기획서 2.2
TARGETS = {
    6: ('E2', 82.4069),
    5: ('A2', 110.0000),
    2: ('B3', 246.9417),
    1: ('E4', 329.6276),
}
STRING_ORDER = [6, 5, 2, 1]


def error_cents(measured_hz: float, target_hz: float) -> float:
    """센트 오차 = 1200 * log2(측정/목표)."""
    return 1200.0 * math.log2(measured_hz / target_hz)


def is_in_tune(measured_hz: float, target_hz: float, tol_cents: float) -> bool:
    return abs(error_cents(measured_hz, target_hz)) <= tol_cents


def min_step_cents(cal) -> float:
    """
    이 줄에서 낼 수 있는 가장 작은 음정 변화량(센트).

    허용 센트가 이 값보다 작으면 원리상 수렴할 수 없다. task_manager 가 시작 시 경고한다.
    """
    return cal.min_effective_deg * cal.cents_per_deg


def peg_angle(err_cents: float, cal) -> tuple:
    """
    오차 -> (direction, angle_deg). cal 은 peg_calibration.StringCalibration.

    direction 은 AdjustPeg.Goal.direction 규약을 따른다: +1 조임(음 높이기) / -1 풀림.
    angle_deg 는 항상 양수 크기.

    회전량은 실측 상수 cents_per_deg 로만 계산한다. 코드에 계수를 두지 않는다.
    실측 전에는 호출자가 CalibrationMissing 을 먼저 만나므로 여기까지 오지 않는다.
    """
    direction = 1 if err_cents < 0 else -1   # 음이 낮으면 높여야 함
    angle = abs(err_cents) / cal.cents_per_deg

    # 최소 유효 각도보다 작으면 페그가 실제로 움직이지 않는다.
    # ⚠ 미확정(브리핑 A3-3): 지금은 최소 각도로 올려서 보낸다. 대안은 '조절 포기'다.
    #    실측 전이라 이 경로는 아직 실행되지 않는다. 확정되면 여기만 고치면 된다.
    if angle < cal.min_effective_deg:
        angle = cal.min_effective_deg

    return direction, min(angle, cal.max_angle_deg)


def adjust_plan(err_cents: float, cal) -> list:
    """
    오차 -> AdjustPeg goal 목록. 백래시 마무리를 여기서 결정한다 (CLAUDE.md 3.2).

    확정 사항이라 문서에서 그대로 옮긴 규칙:
      - 최종 접근은 항상 조이는 방향이다.
      - 음이 낮으면(조여야 하면) 이미 조이는 방향이므로 goal 1개로 끝난다.
      - 음이 높으면 목표보다 backlash_release_cents 만큼 아래로 풀었다가 조여 올린다.
        goal 2개가 되며, 이 묶음 전체가 '조절 1회'다 (CLAUDE.md 3.2 "조절 1회가
        adjust_peg goal 2번이 될 수 있다").
      - 두 goal 사이에는 재측정하지 않는다. 묶음이 끝난 뒤 한 번 잰다.

    풀기 각도가 max_angle_deg 에 잘리면 조이기 goal 을 붙이지 않고 그 회차를 끝낸다.
    목표 아래에 머물게 되지만 다음 회차가 조이는 방향으로 다시 잡으므로
    '최종 접근은 조이는 방향' 규칙은 깨지지 않는다.
    """
    if err_cents < 0:
        return [peg_angle(err_cents, cal)]

    release_cents = err_cents + cal.backlash_release_cents
    release_dir, release_deg = peg_angle(release_cents, cal)
    plan = [(release_dir, release_deg)]

    clamped = release_deg >= cal.max_angle_deg
    if not clamped:
        # 푼 만큼 중 목표 아래로 내려간 몫만 조여 올린다
        plan.append(peg_angle(-cal.backlash_release_cents, cal))
    return plan


# ---------------------------------------------------------------------------
# 작업 중 자동 보정 (2026-09-27 사용자 결정 — '작업 중 자동 보정 + 기록')
#
# peg_cal 원자료를 다시 보니 2·1번은 yaml 의 cents_per_deg 만큼 음이 안 바뀐다
# (2번 1.065 에 대해 반복 조임 0.76, 1번 0.502 에 대해 0.40 — 71%·80%). 그래서
# "24도 조였는데 더 조여야 한다" 가 난다. 5번은 1.22~1.27 로 yaml(1.238)과 맞다.
# 원인은 줄마다 다르고(고무 골무 비틀림·재파지 여유) 날마다 흔들려 yaml 값 하나로는
# 못 잡는다. 그래서 조절할 때마다 실제로 변한 센트를 보고 그 줄의 효율(gain)을
# 작업 안에서 고쳐 다음 조절에 쓴다. yaml 은 바꾸지 않는다 — 기록이 쌓이면 사람이 고친다.
#
# ⚠ 아래 숫자는 전부 실측 전 임시값이다 (CLAUDE.md 11절).
GAIN_MIN = 0.5                  # 효율은 yaml 값의 0.5~1.5배 안에서만 움직인다
GAIN_MAX = 1.5
GAIN_ALPHA = 0.5                # 새 관측에 주는 무게. 1 이면 직전 한 번만 믿는다
GAIN_MIN_OBSERVED_CENTS = 3.0   # 이보다 작은 변화는 측정 흔들림(±1~2센트)과 못 가른다


def effective_cal(cal, gain: float):
    """효율을 곱한 캘리브레이션. cents_per_deg 만 바뀌고 나머지 한계는 그대로다."""
    return dataclasses.replace(cal, cents_per_deg=cal.cents_per_deg * float(gain))


def learn_gain(gain: float, cal, plan: list, err_before: float, err_after: float) -> tuple:
    """
    조절 1회의 결과로 효율을 고친다. 반환 (새 gain, 사람이 읽을 사유).

    cal 은 yaml 그대로의 값이다(효율을 곱하기 전). 관측 효율 = 실제 변화 / (cal 기울기 × 각도).
    배우는 건 '조임 한 번' 짜리 조절뿐이다 — 풀기→조임 두 단계는 백래시가 섞여서
    각도와 음 변화의 비를 가를 수 없다.
    """
    if len(plan) != 1 or plan[0][0] <= 0:
        return gain, '건너뜀: 풀기→조임 조절은 백래시가 섞여 배우지 않는다'
    deg = float(plan[0][1])
    observed = float(err_after) - float(err_before)      # 조이면 음이 올라가 + 가 된다
    if observed < GAIN_MIN_OBSERVED_CENTS:
        return gain, '건너뜀: 변화 %+.1f센트는 측정 흔들림과 구분이 안 된다' % observed
    ratio = observed / (cal.cents_per_deg * deg)
    new = (1.0 - GAIN_ALPHA) * float(gain) + GAIN_ALPHA * ratio
    new = min(GAIN_MAX, max(GAIN_MIN, new))
    return new, '관측 효율 %.2f (예상의 %.0f%%) → gain %.2f' % (ratio, ratio * 100.0, new)


def overall_result(results: list) -> int:
    """
    줄별 StringResult.RESULT_* -> 전체 TuningState.OVERALL_*.

    아직 처리 전인 줄이 하나라도 있으면 판정하지 않고 OVERALL_PENDING.
    """
    if not results or any(r == StringResult.RESULT_PENDING for r in results):
        return TuningState.OVERALL_PENDING
    ok = sum(1 for r in results if r == StringResult.RESULT_SUCCESS)
    if ok == len(results):
        return TuningState.OVERALL_SUCCESS
    if ok == 0:
        return TuningState.OVERALL_FAILED
    return TuningState.OVERALL_PARTIAL
