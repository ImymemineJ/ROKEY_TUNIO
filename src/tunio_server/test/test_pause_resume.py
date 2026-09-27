"""실제 ROS 서비스/Action 왕복으로 일시정지와 오디오 폐기를 검증한다."""
import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from tunio_interfaces.srv import CommandTuning
from tunio_server import task_manager as TM
from tunio_server.fake_hardware import FakeHardware


def wait(future):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    assert done.wait(4)
    return future.result()


@pytest.mark.parametrize('command', ['resume', 'cancel'])
def test_robot_action_pauses_in_flight_and_discards_interrupted_audio(command):
    started = threading.Event()
    counts = []

    class RecordedHardware(FakeHardware):
        def on_play(self, goal):
            counts.append(goal.request.preparation_id)
            started.set()
            return super().on_play(goal)

    rclpy.init()
    hardware = RecordedHardware()
    manager = TM.TaskManager()
    client_node = rclpy.create_node('pause_test_client')
    executor = MultiThreadedExecutor(num_threads=6)
    results, errors = [], []
    for node in (hardware, manager, client_node):
        executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    worker = None
    try:
        hardware.set_parameters([
            Parameter('ready_delay_sec', value=.01),
            Parameter('play_delay_sec', value=.15),
            Parameter('measure_delay_sec', value=.02)])
        manager.set_parameters([Parameter('play_timeout_sec', value=.3)])
        manager.active_settings = manager.settings
        manager.job_id = 'pause-test'
        for action in (manager.cli_prepare, manager.cli_measure, manager.cli_play):
            assert action.wait_for_server(timeout_sec=3)
        for service in (manager.cli_pause, manager.cli_resume):
            assert service.wait_for_service(timeout_sec=3)
        commands = {name: client_node.create_client(CommandTuning, '/tuning/' + name)
                    for name in ('pause', command)}
        for service in commands.values():
            assert service.wait_for_service(timeout_sec=3)

        def measure():
            try:
                results.append(manager._measure_once(6))
            except Exception as exc:
                errors.append(exc)
        worker = threading.Thread(target=measure)
        manager._worker = worker
        worker.start()
        assert started.wait(3)
        assert wait(commands['pause'].call_async(CommandTuning.Request())).accepted
        assert manager.phase == TM.TuningState.PHASE_PAUSED
        time.sleep(.4)  # 실제 Action 예산 .3초보다 오래 멈춰도 오류가 나면 안 된다.
        assert worker.is_alive() and len(counts) == 1 and not errors
        assert wait(commands[command].call_async(CommandTuning.Request())).accepted
        worker.join(4)
        assert not worker.is_alive()
        if command == 'resume':
            assert not errors and results[0].ok
            assert len(counts) == 2 and counts[0] != counts[1]
            assert manager.measurement_id == 2
        else:
            assert not results and isinstance(errors[0], TM.Cancelled)
            assert len(counts) == 1
    finally:
        manager._cancel.set()
        manager._pause.clear()
        hardware._paused.clear()
        manager._request_goal_cancel()
        if worker:
            worker.join(4)
        executor.shutdown()
        spin.join(3)
        for node in (hardware, manager, client_node):
            node.destroy_node()
        rclpy.shutdown()
