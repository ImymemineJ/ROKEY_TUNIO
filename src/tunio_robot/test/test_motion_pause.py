"""실제 드라이버 호출 없이 이동 중 pause/resume, 취소 우선, 실패 차단을 검증한다."""
from concurrent.futures import Future
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tunio_robot.motion_cancel import CancelSession, GuardedAPI, MotionCancelled
from tunio_robot.motion_pause import PauseControl


def client(function):
    def send(request):
        future = Future()
        future.set_result(function(request))
        return future
    return SimpleNamespace(wait_for_service=lambda timeout_sec=None: True, call_async=send)


def control():
    state = SimpleNamespace(motion=0, robot=1, paused=False, pause_calls=0, resume_calls=0)

    def pause(_):
        state.paused = True
        state.pause_calls += 1
        return SimpleNamespace(success=True)

    def resume(_):
        state.paused = False
        state.resume_calls += 1
        return SimpleNamespace(success=True)
    ctrl = PauseControl(client(pause), client(resume),
                        client(lambda _: SimpleNamespace(success=True, status=state.motion)),
                        client(lambda _: SimpleNamespace(success=True, robot_state=state.robot)),
                        .2, .001)
    return ctrl, state


def test_pause_interrupts_async_move_and_resume_does_not_send_move_twice():
    ctrl, state = control()
    session = CancelSession()
    started = threading.Event()
    raw = Mock()
    target = [10., 0., 0., 0., 0., 0.]
    raw.get_current_posj.return_value = [0.] * 6

    def move(*args, **kw):
        state.motion = 2
        started.set()
        return 0  # 두산의 접수 성공 반환값. 아직 도착한 것은 아니다.
    raw.amovej.side_effect = move
    api = GuardedAPI(raw, session.check, ctrl)
    finished = threading.Event()

    def work():
        api.movej(target, vel=5, acc=10)
        api.set_digital_output(1, 1)
        finished.set()
    thread = threading.Thread(target=work)
    thread.start()
    try:
        assert started.wait(1)
        ctrl.pause()
        assert state.pause_calls == 1
        assert state.paused and not finished.wait(.03)
        raw.set_digital_output.assert_not_called()
        ctrl.resume(session.check)
        assert state.resume_calls == 1
        raw.get_current_posj.return_value = target
        state.motion = 0
        assert finished.wait(1)
        raw.amovej.assert_called_once_with(target, vel=5, acc=10)
        raw.movej.assert_not_called()
        raw.set_digital_output.assert_called_once()
    finally:
        session.requested.set()
        thread.join(1)


@pytest.mark.parametrize('robot_state', [0, 3, 4, 5, 6, 7, 8, 9, 10, 15, 99])
def test_resume_rejects_protective_or_unknown_states(robot_state):
    ctrl, state = control()
    ctrl.paused.set()
    ctrl.driver_paused = True
    state.robot = robot_state
    with pytest.raises(RuntimeError):
        ctrl.resume(lambda: None)
    assert state.resume_calls == 0 and ctrl.paused.is_set()


def test_cancel_while_paused_never_resumes():
    ctrl, state = control()
    session = CancelSession()
    ctrl.paused.set()
    ctrl.driver_paused = True
    session.requested.set()
    with pytest.raises(MotionCancelled):
        ctrl.resume(session.check)
    with pytest.raises(MotionCancelled):
        ctrl.gate(session.check)
    assert state.resume_calls == 0


def test_idle_pause_blocks_commands_without_calling_driver_pause():
    ctrl, state = control()
    ctrl.pause()
    assert state.pause_calls == 0 and ctrl.paused.is_set()
    ctrl.resume(lambda: None)
    assert state.resume_calls == 0 and not ctrl.paused.is_set()


def test_idle_after_acceptance_does_not_mean_target_reached():
    ctrl, state = control()
    ctrl.timeout = .01
    raw = Mock(amovej=Mock(return_value=0), get_current_posj=Mock(return_value=[0.] * 6))
    with pytest.raises(RuntimeError, match='확인 불가'):
        ctrl.invoke(raw, lambda: None, 'movej', ([20.] * 6,), {})
    assert ctrl.failed
    with pytest.raises(RuntimeError):
        ctrl.invoke(raw, lambda: None, 'set_digital_output', (1, 1), {})
    raw.set_digital_output.assert_not_called()


def test_short_move_can_finish_between_status_polls():
    ctrl, _ = control()
    raw = Mock(amovej=Mock(return_value=0), get_current_posj=Mock(return_value=[20.] * 6))
    assert ctrl.invoke(raw, lambda: None, 'movej', ([20.] * 6,), {}) == 0


def test_equivalent_zyz_angles_and_relative_joint_target():
    ctrl, _ = control()
    raw = Mock(get_current_posx=Mock(return_value=([0., 0., 0., 30., 0., 20.], 0)),
               get_current_posj=Mock(return_value=[1.] * 6))
    assert ctrl.reached(raw, 'movel', [0., 0., 0., 0., 0., 50.])
    assert ctrl.target(raw, 'movej', ([2.] * 6,), {'mod': 1}) == [3.] * 6


def test_blended_stroke_keeps_existing_synchronous_call():
    ctrl, _ = control()
    raw = Mock(movel=Mock(return_value=0))
    assert ctrl.invoke(raw, lambda: None, 'movel', ([0.] * 6,), {'radius': 3}) == 0
    raw.amovel.assert_not_called()


def test_force_control_pause_cannot_claim_position_hold():
    ctrl, _ = control()
    ctrl.force_active = True
    with pytest.raises(RuntimeError, match='힘 제어'):
        ctrl.pause()
