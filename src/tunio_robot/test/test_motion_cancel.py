"""실제 로봇 없이 MoveStop 요청·IDLE 판정·후속 명령 차단을 검증한다."""
from concurrent.futures import Future
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tunio_robot import motion_primitives as MP
from tunio_robot.motion_cancel import (
    CancelSession, GuardedAPI, MotionCancelled, request_and_confirm_stop)
from tunio_robot.robot_controller import RobotController


def client(responses):
    calls = []
    responses = iter(responses)

    def send(req):
        calls.append(req)
        f = Future()
        value = next(responses)
        if value is not None:
            f.set_result(value)
        return f
    return SimpleNamespace(wait_for_service=lambda timeout_sec=None: True, call_async=send), calls


def bind(node):
    """가짜 노드(SimpleNamespace)에 on_goal 이 부르는 메서드 둘을 묶어 준다."""
    node._notify = lambda message, error=True: RobotController._notify(node, message, error)
    node._reject = lambda reason: RobotController._reject(node, reason)
    return node


def test_stop_mode_2_and_busy_then_idle():
    session = CancelSession()
    session.work_done.set()
    stop, calls = client([SimpleNamespace(success=True)] * 2)
    check, _ = client([SimpleNamespace(success=True, status=s) for s in (2, 0)])
    request_and_confirm_stop(stop, check, session, 1, .001)
    assert session.stop_done.is_set()
    assert session.error is None
    assert [r.stop_mode for r in calls] == [2, 2]


@pytest.mark.parametrize('kind', ['stop_failure', 'unknown', 'timeout', 'work_running'])
def test_failed_or_unconfirmed_stop_is_not_success(kind):
    session = CancelSession()
    if kind != 'work_running':
        session.work_done.set()
    stop, _ = client([SimpleNamespace(success=kind != 'stop_failure')] * 100)
    check, _ = client([None if kind == 'timeout' else
                      SimpleNamespace(success=True, status=99)] * 100)
    request_and_confirm_stop(stop, check, session, .02, .001)
    assert session.stop_done.is_set()
    assert session.error


def test_cancel_during_blocking_move_prevents_grip_and_cleanup_retreat():
    session = CancelSession()
    entered = threading.Event()
    stopped = threading.Event()
    raw = Mock()

    def move():
        entered.set()
        assert stopped.wait(1)
        return 0  # 두산 래퍼 성공 반환 규약
    raw.movej.side_effect = move
    api = GuardedAPI(raw, session.check)
    errors = []

    def work():
        try:
            api.movej()
            api.set_digital_output(1, 1)
        except Exception as exc:
            errors.append(exc)
            with pytest.raises(MotionCancelled):
                api.movel()
            api.release_force()
    thread = threading.Thread(target=work)
    thread.start()
    assert entered.wait(1)
    session.requested.set()
    stopped.set()
    thread.join(1)
    assert isinstance(errors[0], MotionCancelled)
    raw.set_digital_output.assert_not_called()
    raw.movel.assert_not_called()
    raw.release_force.assert_called_once()


def test_robot_cancel_stops_and_latches_recovery(monkeypatch):
    # MoveStop 대역이 기존 블로킹 이동을 풀어 주는 상황. 실제 API 호출 없음.
    entered = threading.Event()
    stopped = threading.Event()
    stop_calls = []

    def stop_request(stop, check, session, timeout, interval):
        stop_calls.append(True)
        stopped.set()
        assert session.work_done.wait(1)
        session.stop_done.set()
    monkeypatch.setattr('tunio_robot.robot_controller.request_and_confirm_stop', stop_request)
    monkeypatch.setattr(MP, '_cancel_check', None)
    node = bind(SimpleNamespace(
        _lock=threading.Lock(), _running=None, _cancel_sessions={},
        _recovery_required=False, _prepared_pluck=None, _holding_pick=True, _pick_fault=None,
        dry_run=False, _stop_timeout=1, _stop_interval=.001, _service_wait=.01,
        _stop_client=Mock(), _motion_client=Mock(), _notice_pub=Mock(),
        get_logger=lambda: Mock(), _record=Mock()))
    goal = Mock(is_active=True)
    goal.canceled.side_effect = lambda: setattr(goal, 'is_active', False)

    def result_type(**kw):
        return SimpleNamespace(**kw)

    results = []

    def work():
        entered.set()
        assert stopped.wait(1)
        MP._cancel_check()
    worker = threading.Thread(target=lambda: results.append(
        RobotController._run(node, 'play_string', goal, result_type, work)))
    worker.start()
    assert entered.wait(1)
    RobotController.on_cancel(node, goal)
    worker.join(2)
    assert not worker.is_alive()
    assert stop_calls == [True]
    goal.canceled.assert_called_once()
    goal.succeed.assert_not_called()
    assert not results[0].success
    assert node._holding_pick is None
    assert node._recovery_required
    assert RobotController.on_goal(node, None).name == 'REJECT'
    assert RobotController.on_cancel(node, goal).name == 'REJECT'
    # 거부 사유가 /robot/notice 로 나간다. 서버는 이걸로 '왜' 거부됐는지 화면에 올린다
    notice = node._notice_pub.publish.call_args[0][0].data
    assert '복구' in notice and '재시작' in notice


def test_goal_rejected_when_stop_services_never_match_and_reason_is_published():
    """정지/상태 서비스가 대기 시간 안에 안 붙으면 거부하고 사유를 발행한다.

    한 번 보고 마는 service_is_ready() 대신 wait_for_service 로 기다린다 —
    2026-09-23 실기에서 목록엔 보이던 서비스가 그 순간 매칭이 안 돼 거부됐다.
    """
    waits = []

    def never(timeout_sec):
        waits.append(timeout_sec)
        return False
    node = bind(SimpleNamespace(
        _lock=threading.Lock(), _running=None, _recovery_required=False, dry_run=False,
        _service_wait=.25, _stop_client=SimpleNamespace(wait_for_service=never),
        _motion_client=Mock(), _notice_pub=Mock(), get_logger=lambda: Mock()))
    assert RobotController.on_goal(node, None).name == 'REJECT'
    assert waits == [.25]
    assert '미연결' in node._notice_pub.publish.call_args[0][0].data


def test_goal_accepted_once_stop_services_match_within_wait():
    node = bind(SimpleNamespace(
        _lock=threading.Lock(), _running=None, _recovery_required=False, dry_run=False,
        _service_wait=.25,
        _stop_client=SimpleNamespace(wait_for_service=lambda timeout_sec: True),
        _motion_client=SimpleNamespace(wait_for_service=lambda timeout_sec: True),
        _notice_pub=Mock(), get_logger=lambda: Mock()))
    assert RobotController.on_goal(node, None).name == 'ACCEPT'
    node._notice_pub.publish.assert_not_called()


def test_stop_request_waits_for_service_instead_of_single_check():
    """정지(MoveStop) 클라이언트가 아직 매칭 전이어도 deadline 안에 붙으면 정지를 보낸다."""
    session = CancelSession()
    session.work_done.set()
    attempts = []

    def late(timeout_sec):
        attempts.append(timeout_sec)
        return len(attempts) >= 2
    calls = []

    def send(req):
        calls.append(req)
        f = Future()
        f.set_result(SimpleNamespace(success=True, status=0))
        return f
    stop = SimpleNamespace(wait_for_service=late, call_async=send)
    check = SimpleNamespace(wait_for_service=lambda timeout_sec: True, call_async=send)
    request_and_confirm_stop(stop, check, session, 1, .001)
    # 첫 wait 은 False 였지만 두 번째에 붙어 정지가 나갔다 — 한 번 보고 포기하지 않는다
    assert session.error is None
    assert len(attempts) >= 2 and len(calls) >= 2


def test_ros_cancel_reaches_canceled_result_without_real_robot():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from tunio_interfaces.action import ReturnHome

    rclpy.init()
    robot = RobotController()
    client_node = rclpy.create_node('cancel_test_client')
    executor = MultiThreadedExecutor(num_threads=4)
    entered = threading.Event()
    release = threading.Event()

    def hold():
        entered.set()
        assert release.wait(3)
    robot._return_to_home = hold
    # Action 자체는 실제 ROS로 왕복하고 work만 대역으로 교체한다.
    original_run = robot._run
    robot._run = lambda name, goal, result_type, work: original_run(
        name, goal, result_type, hold)
    executor.add_node(robot)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        client = ActionClient(client_node, ReturnHome, '/robot/return_home')
        assert client.wait_for_server(timeout_sec=3)
        accepted = client.send_goal_async(ReturnHome.Goal())
        event = threading.Event()
        accepted.add_done_callback(lambda _: event.set())
        assert event.wait(3)
        goal = accepted.result()
        assert goal.accepted and entered.wait(3)
        future = goal.get_result_async()
        cancel = goal.cancel_goal_async()
        event.clear()
        cancel.add_done_callback(lambda _: event.set())
        assert event.wait(3)
        assert cancel.result().goals_canceling
        release.set()
        event.clear()
        future.add_done_callback(lambda _: event.set())
        assert event.wait(3)
        assert future.result().status == 5
        assert not future.result().result.success
    finally:
        release.set()
        executor.shutdown()
        thread.join(3)
        robot.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
