"""일시정지/재개 요청이 로봇에 닿기 전과 닿은 뒤의 실패를 다르게 다룬다.

2026-09-23 실기에서 '일시정지 확인 실패: 로봇 일시정지/재개 서비스 미연결' 로 작업이
통째로 오류 종료됐다. 요청을 보내지도 못한 실패는 로봇 상태를 바꾸지 않으므로
거절만 하고 작업은 계속 돌아야 한다. 보낸 뒤 무응답·거부만 취소로 전환한다.
"""
import threading
from types import SimpleNamespace as Namespace
from unittest.mock import Mock

import pytest

from tunio_interfaces.msg import TuningState
from tunio_server import task_manager as TM


def manager(control=None, running=True, confirmed=False):
    n = Namespace(
        _control_lock=threading.Lock(), _running=lambda: running,
        _cancel=threading.Event(), _pause=threading.Event(), _pause_epoch=0,
        _pause_confirmed=confirmed, _phase_before_pause=TuningState.PHASE_IDLE,
        phase=TuningState.PHASE_PLUCK, _control_error='',
        _publish_state=Mock(), _request_goal_cancel=Mock(), get_logger=lambda: Mock(),
        cli_pause=Mock(name='cli_pause'), cli_resume=Mock(name='cli_resume'))
    n._robot_control = control if control is not None else Mock()
    n._control_lost = lambda what, exc: TM.TaskManager._control_lost(n, what, exc)
    return n


def response():
    return Namespace(accepted=False, message='')


def test_pause_confirmed_marks_paused_and_bumps_epoch():
    n = manager()
    res = TM.TaskManager.on_pause(n, None, response())
    assert res.accepted
    assert n._pause.is_set() and n._pause_confirmed
    assert n.phase == TuningState.PHASE_PAUSED
    assert n._phase_before_pause == TuningState.PHASE_PLUCK
    assert n._pause_epoch == 1
    assert not n._cancel.is_set()


def test_pause_unreachable_is_refused_and_job_keeps_running():
    """서비스에 요청이 안 갔으면 거절만 한다. 취소·측정 폐기·오류 종료로 번지지 않는다."""
    n = manager(control=Mock(side_effect=TM.ControlUnreachable('미연결')))
    res = TM.TaskManager.on_pause(n, None, response())
    assert not res.accepted and '미연결' in res.message
    assert not n._pause.is_set()          # 작업 스레드가 다음 단계로 계속 간다
    assert not n._pause_confirmed
    assert n._pause_epoch == 0            # 진행 중이던 측정을 폐기하지 않는다
    assert not n._cancel.is_set() and n._control_error == ''
    n._request_goal_cancel.assert_not_called()
    assert n.phase == TuningState.PHASE_PLUCK


@pytest.mark.parametrize('failure', [TM.Aborted('로봇 일시정지 응답 없음'),
                                     RuntimeError('예상 못 한 예외')])
def test_pause_failure_after_send_turns_into_cancel(failure):
    """보낸 뒤 실패는 로봇이 멈췄는지 모르는 상태다. 취소로 전환하고 goal 을 끊는다."""
    n = manager(control=Mock(side_effect=failure))
    res = TM.TaskManager.on_pause(n, None, response())
    assert not res.accepted
    assert n._cancel.is_set()
    assert n._control_error.startswith('일시정지 확인 실패')
    assert not n._pause.is_set()
    n._request_goal_cancel.assert_called_once()


def test_resume_unreachable_keeps_paused_state():
    """재개 요청이 안 갔으면 로봇은 여전히 멈춰 있다. 일시정지를 유지하고 다시 누르게 한다."""
    n = manager(control=Mock(side_effect=TM.ControlUnreachable('미연결')), confirmed=True)
    n._pause.set()
    n.phase = TuningState.PHASE_PAUSED
    res = TM.TaskManager.on_resume(n, None, response())
    assert not res.accepted and '일시정지 유지' in res.message
    assert n._pause.is_set() and n._pause_confirmed
    assert n.phase == TuningState.PHASE_PAUSED
    assert not n._cancel.is_set()


def test_resume_failure_after_send_turns_into_cancel():
    n = manager(control=Mock(side_effect=TM.Aborted('거부')), confirmed=True)
    n._pause.set()
    res = TM.TaskManager.on_resume(n, None, response())
    assert not res.accepted
    assert n._cancel.is_set() and n._control_error.startswith('재개 확인 실패')
    n._request_goal_cancel.assert_called_once()


def test_robot_control_waits_for_service_then_sends():
    """service_is_ready() 한 번이 아니라 node_wait_sec 동안 기다린다."""
    waited = []

    def wait_for_service(timeout_sec):
        waited.append(timeout_sec)
        return True
    client = Namespace(wait_for_service=wait_for_service,
                       call_async=lambda req: 'future')
    n = Namespace(_p=lambda key: {'node_wait_sec': 5.0, 'robot_control_timeout_sec': 20.0}[key],
                  _wait=Mock(return_value=Namespace(accepted=True, message='')))
    TM.TaskManager._robot_control(n, client, '일시정지')
    assert waited == [5.0]
    n._wait.assert_called_once()


def test_robot_control_unreachable_when_service_never_matches():
    client = Namespace(wait_for_service=lambda timeout_sec: False, call_async=Mock())
    n = Namespace(_p=lambda key: 5.0, _wait=Mock())
    with pytest.raises(TM.ControlUnreachable, match='미연결'):
        TM.TaskManager._robot_control(n, client, '일시정지')
    client.call_async.assert_not_called()


@pytest.mark.parametrize('reply', [None, Namespace(accepted=False, message='힘 제어 중')])
def test_robot_control_after_send_failure_is_aborted(reply):
    client = Namespace(wait_for_service=lambda timeout_sec: True, call_async=lambda req: 'f')
    n = Namespace(_p=lambda key: 5.0, _wait=Mock(return_value=reply))
    with pytest.raises(TM.Aborted):
        TM.TaskManager._robot_control(n, client, '재개')


def test_rejected_goal_message_carries_robot_notice():
    """로봇이 /robot/notice 로 보낸 거부 사유가 오류 문구에 실려 화면까지 간다."""
    n = Namespace(_gate=Mock(), _wait=lambda *a: Namespace(accepted=False), _p=lambda key: 5,
                  _robot_notice='새 동작 거부 — 취소/제어 실패 후 현장 복구 확인·robot_controller 재시작 필요')
    with pytest.raises(TM.Aborted, match='goal 거부.*재시작 필요'):
        TM.TaskManager._run_action(n, Mock(), object(), 120, 'register_peg')


def test_ready_wait_reacts_to_cancel_before_ready_timeout():
    """마이크 ready 대기 중 취소하면 ready_wait_sec 를 다 안 기다리고 빠진다."""
    import time
    handle = Mock(accepted=True)
    result_future = Mock(done=lambda: True)
    handle.get_result_async.return_value = result_future
    send = Mock()
    cancel = threading.Event()
    n = Namespace(
        _gate=Mock(), _cancel=cancel, _pause=threading.Event(), job_id='j', measurement_id=0,
        active_settings=Namespace(measure_timeout_sec=10.0, onset_timeout_sec=10.0),
        _p=lambda key: {'prepare_timeout_sec': 30.0, 'goal_response_sec': 5.0,
                        'ready_wait_sec': 10.0}[key],
        _run_action=Mock(return_value=Namespace(success=True, preparation_id='p1')),
        cli_prepare=Mock(),
        _set_phase=Mock(), _track=Mock(), _untrack=Mock(), get_logger=lambda: Mock(),
        cli_measure=Namespace(send_goal_async=lambda goal, feedback_callback: send),
        _wait=Mock(return_value=handle))
    threading.Timer(0.2, cancel.set).start()
    t0 = time.monotonic()
    with pytest.raises(TM.Cancelled):
        TM.TaskManager._measure_attempt(n, 6)
    assert time.monotonic() - t0 < 5.0
    handle.cancel_goal_async.assert_called()
