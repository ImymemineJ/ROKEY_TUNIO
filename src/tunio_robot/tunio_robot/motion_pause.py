"""이동 전송과 pause/resume을 직렬화한다. DSR 래퍼를 여러 스레드에서 spin하지 않는다."""
import math
import threading
import time

from tunio_robot.motion_cancel import wait_ready


def service_call(client, request, timeout):
    if not wait_ready(client, timeout):
        raise RuntimeError('로봇 제어 서비스 미연결 (%.1f초 대기)' % timeout)
    future = client.call_async(request)
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout):
        raise RuntimeError('로봇 제어 서비스 응답 시간 초과')
    response = future.result()
    if response is None or not response.success:
        raise RuntimeError('로봇 제어 서비스 실패 응답')
    return response


class PauseControl:
    def __init__(self, pause, resume, motion, state, timeout, interval,
                 position_tolerance=.5, angle_tolerance=.1):
        self.clients = pause, resume, motion, state
        self.timeout = timeout
        self.interval = interval
        self.position_tolerance = position_tolerance
        self.angle_tolerance = angle_tolerance
        self.lock = threading.RLock()
        self.paused = threading.Event()
        self.driver_paused = False
        self.failed = False
        self.force_active = False
        self.moving = False
        self.resume_epoch = 0
        if not all(math.isfinite(v) and v > 0 for v in
                   (timeout, interval, position_tolerance, angle_tolerance)):
            raise ValueError('제어 시간·주기·도달 허용 오차는 유한한 양수여야 한다')

    def gate(self, check):
        while True:
            check()
            if self.failed:
                raise RuntimeError('일시정지/재개 실패 — 현장 복구 필요')
            if not self.paused.is_set():
                return
            time.sleep(self.interval)

    def status(self):
        from dsr_msgs2.srv import CheckMotion
        value = service_call(self.clients[2], CheckMotion.Request(), self.timeout).status
        if value not in (0, 1, 2):
            raise RuntimeError(f'알 수 없는 이동 상태: {value}')
        return value

    def healthy(self):
        from dsr_msgs2.srv import GetRobotState
        value = service_call(self.clients[3], GetRobotState.Request(), self.timeout).robot_state
        # 보호정지뿐 아니라 교시·복구·미확인 상태에서도 자동 재개하지 않는다.
        if value not in (1, 2):
            raise RuntimeError(f'재개/이동 완료를 확인할 수 없는 로봇 상태: {value}')

    def pause(self):
        from dsr_msgs2.srv import MovePause
        self.paused.set()  # 새로운 이동/I/O 진입부터 막는다.
        with self.lock:
            if self.failed:
                raise RuntimeError('제어 실패 후 현장 복구 필요')
            if self.force_active:
                raise RuntimeError('힘 제어 중에는 자세 유지 재개를 보장할 수 없어 취소로 전환')
            if self.driver_paused:
                return
            self.healthy()
            status = self.status()
            if status in (1, 2) or self.moving:
                service_call(self.clients[0], MovePause.Request(), self.timeout)
                self.driver_paused = True

    def resume(self, check):
        from dsr_msgs2.srv import MoveResume
        with self.lock:
            check()
            if self.failed:
                raise RuntimeError('제어 실패 후 재개 금지')
            self.healthy()
            check()
            if self.driver_paused:
                service_call(self.clients[1], MoveResume.Request(), self.timeout)
            check()
            self.driver_paused = False
            self.resume_epoch += 1
            self.paused.clear()

    @staticmethod
    def vector(value):
        values = list(value)
        if len(values) != 6 or not all(math.isfinite(float(v)) for v in values):
            raise RuntimeError('이동 목표/현재 좌표 조회 실패')
        return values

    def target(self, api, name, args, kwargs):
        target = self.vector(args[0])
        if kwargs.get('mod', 0) == 1:
            if name == 'movej':
                current = self.vector(api.get_current_posj())
                target = [a + b for a, b in zip(current, target)]
            else:
                current = self.vector(api.get_current_posx(ref=0)[0])
                target = self.vector(api.trans(current, target, ref=kwargs.get('ref', 0),
                                               ref_out=0))
        elif kwargs.get('ref', 0) != 0:
            raise RuntimeError('Base 이외의 절대 목표는 완료 검증을 지원하지 않는다')
        return target

    def reached(self, api, name, target):
        if name == 'movej':
            current = self.vector(api.get_current_posj())
            # J6 누적각을 360도로 환산해 목표 도달로 오판하지 않는다.
            return max(abs(a - b) for a, b in zip(current, target)) <= self.angle_tolerance
        current = self.vector(api.get_current_posx(ref=0)[0])
        distance = math.dist(current[:3], target[:3])
        # ZYZ의 같은 자세가 다른 오일러각으로 반환될 수 있어 회전행렬로 비교한다.

        def rotation(pose):
            a, b, c = map(math.radians, pose[3:])
            ca, sa, cb, sb, cc, sc = (math.cos(a), math.sin(a), math.cos(b),
                                      math.sin(b), math.cos(c), math.sin(c))
            return (ca*cb*cc-sa*sc, -ca*cb*sc-sa*cc, ca*sb,
                    sa*cb*cc+ca*sc, -sa*cb*sc+ca*cc, sa*sb, -sb*cc, sb*sc, cb)
        trace = sum(a*b for a, b in zip(rotation(current), rotation(target)))
        angle = math.degrees(math.acos(max(-1., min(1., (trace - 1.) / 2.))))
        return distance <= self.position_tolerance and angle <= self.angle_tolerance

    def invoke(self, api, check, name, args, kwargs):
        while True:
            self.gate(check)
            with self.lock:
                check()
                if self.paused.is_set():
                    continue
                # 비동기 래퍼는 radius를 0으로 바꾼다. 발현 radius=3은 원래 호출 유지.
                asynchronous = name in ('movej', 'movejx', 'movel') and not kwargs.get('radius')
                try:
                    target = self.target(api, name, args, kwargs) if asynchronous else None
                    result = getattr(api, 'a' + name if asynchronous else name)(*args, **kwargs)
                except Exception:
                    self.failed = True
                    raise
                if asynchronous:
                    self.moving = result == 0
                if name == 'set_desired_force' and result == 0:
                    self.force_active = True
                break
        if not asynchronous or result != 0:
            return result
        # IDLE + 정상 상태 + 목표 도달을 함께 확인한다. 짧은 이동도 BUSY 관측에 의존하지 않는다.
        seen_active = False
        epoch = self.resume_epoch
        start_deadline = time.monotonic() + self.timeout
        try:
            while True:
                was_paused = self.paused.is_set()
                self.gate(check)
                if was_paused or epoch != self.resume_epoch:
                    epoch = self.resume_epoch
                    seen_active = False
                    start_deadline = time.monotonic() + self.timeout
                with self.lock:
                    check()
                    if self.paused.is_set():
                        continue
                    status = self.status()
                    seen_active = seen_active or status in (1, 2)
                    if status == 0:
                        self.healthy()
                        if self.reached(api, name, target):
                            self.moving = False
                            return result
                        if seen_active:
                            raise RuntimeError('이동은 종료됐지만 목표 미도달 — 후속 이동 차단')
                    if not seen_active and time.monotonic() >= start_deadline:
                        raise RuntimeError('비동기 이동 시작/완료 확인 불가 — 후속 이동 차단')
                time.sleep(self.interval)
        except Exception:
            self.failed = True
            raise
