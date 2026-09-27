"""Action 취소 가드와 독립 ROS 정지 경로. 실제 이동 명령은 이 모듈에서 보내지 않는다."""
import threading
import time


class MotionCancelled(Exception):
    pass


def wait_ready(client, timeout):
    """서비스가 붙을 때까지 timeout 초 기다린다. 한 번 보고 포기하지 않는다.

    PC 두 대 사이에서는 서비스가 떠 있어도 그 순간 매칭이 안 돼 있을 수 있다.
    2026-09-23 실기에서 목록 조회로는 보이던 서비스가 연결 검사 시점에 '미연결'로
    찍혀 작업이 통째로 중단됐다. 단 한 번 `service_is_ready()` 는 그 틈에 걸린다.
    """
    if client is None:     # 만들지도 않았다 (dry_run 으로 시작한 노드) — 기다릴 것이 없다
        return False
    return bool(client.wait_for_service(timeout_sec=max(0.0, float(timeout))))


class GuardedAPI:
    def __init__(self, api, check, pause=None):
        self.api = api
        self.check = check
        self.pause = pause

    def __getattr__(self, name):
        value = getattr(self.api, name)
        if not callable(value):
            return value

        def call(*args, **kwargs):
            # 취소 후 힘/순응 해제는 허용하되 이동·그리퍼 열기는 허용하지 않는다.
            cleanup = name in ('release_force', 'release_compliance_ctrl')
            if not cleanup:
                self.check()
            if self.pause is not None and not cleanup:
                result = self.pause.invoke(self.api, self.check, name, args, kwargs)
            else:
                result = value(*args, **kwargs)
                if self.pause is not None and name == 'release_force' and result == 0:
                    self.pause.force_active = False
            if not cleanup:
                self.check()
            return result
        return call


class CancelSession:
    def __init__(self):
        self.requested = threading.Event()
        self.work_done = threading.Event()
        self.stop_done = threading.Event()
        self.error = None

    def check(self):
        if self.requested.is_set():
            raise MotionCancelled('사용자 취소 — 후속 명령 차단')


def request_and_confirm_stop(stop, check_motion, session, timeout, interval):
    """stop(mode=2) 성공 + 작업 코드 이탈 + IDLE(0) 확인 전에는 성공하지 않는다.

    명령 전 검사와 실제 서비스 전송 사이에 취소가 끼는 경합을 위해 작업 코드가
    끝날 때까지 stop을 반복한다. 성공 응답만으로 물리 정지를 추정하지 않는다.
    """
    deadline = time.monotonic() + timeout

    def call(client, request):
        # 매칭 확인도 deadline 안에서 되풀이한다. 대기 함수가 기다리지 않고 바로 False 를
        # 돌려주는 경우(매칭 틈·그래프 갱신 전)에도 한 번 보고 포기하지 않는다
        while not wait_ready(client, min(interval * 10, max(0.0, deadline - time.monotonic()))):
            if time.monotonic() >= deadline:
                raise RuntimeError('정지/상태 서비스 연결 안 됨 (%.1f초 대기)' % timeout)
            time.sleep(interval)
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(max(0, deadline - time.monotonic())):
            raise RuntimeError('정지/상태 서비스 응답 시간 초과')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('정지/상태 서비스 실패 응답')
        return response

    try:
        from dsr_msgs2.srv import CheckMotion, MoveStop
        while time.monotonic() < deadline:
            call(stop, MoveStop.Request(stop_mode=2))
            if session.work_done.is_set():
                status = call(check_motion, CheckMotion.Request()).status
                if status == 0:
                    return
                if status not in (1, 2):
                    raise RuntimeError(f'알 수 없는 CheckMotion 상태: {status}')
            time.sleep(min(interval, max(0, deadline - time.monotonic())))
        raise RuntimeError('정지 완료 확인 시간 초과')
    except Exception as exc:
        session.error = str(exc)
    finally:
        session.stop_done.set()
