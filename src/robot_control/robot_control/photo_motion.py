"""
photo_motion.py
PhotoSession 액션 서버 — 사람 중심 4웨이포인트 동적 촬영 모션.

두산 코딩 규칙 §11(멀티스레드 액션 서버) 패턴을 정확히 따른다:
  - perform_task_loop (작업 스레드): DSR API 는 여기서만 호출
  - ros_spin_thread (MultiThreadedExecutor): 액션 서버 스핀
  - trigger_event(threading.Event) 로 스레드 간 신호
  - §18 amovel + check_motion 시간기반 대기 + 진행률 피드백

좌표는 resource/waypoints.yaml 에서 로드한다 (하드코딩 제거).
각 웨이포인트 도착 시 /capture_image (std_srvs/Trigger) 를 호출해
photo_booth_manager 가 RealSense 고화질 캡처를 수행하도록 신호.
"""

import os
import time
import random
import threading

import yaml
import rclpy
import DR_init
from ament_index_python.packages import get_package_share_directory

# ---- 두산 규칙 §1 ----
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor  # noqa: E402
from rclpy.action import ActionServer, GoalResponse         # noqa: E402
from std_srvs.srv import Trigger                            # noqa: E402
from doocut_interfaces.action import PhotoSession           # noqa: E402

NODE_NAME = "photo_motion_server"

# ---- 전역 상태 (§11 패턴) ----
node_ = None
trigger_event = threading.Event()
task_running = False
g_current_goal_handle = None
g_final_result = None
g_goal_request = None

# 캡처 서비스 전용 node/client/executor (작업 스레드에서만 사용)
capture_node_ = None
capture_cli = None
capture_executor_ = None

# ---- waypoints.yaml 에서 로드되는 값 ----
DEFAULT_SUBJECT_CENTER = [525.72, -113.62, 565.21, 65.53, -101.16, -87.38]
WAYPOINTS = []
HOME_JOINT = [0, 0, 90, 0, 90, 0]
VELOCITY = 40
ACC = 40
MOVE_TIME = 5.0


def _load_waypoints():
    """resource/waypoints.yaml 로드. share -> 소스트리 순으로 폴백."""
    candidates = []
    try:
        share = get_package_share_directory("robot_control")
        candidates.append(os.path.join(share, "resource", "waypoints.yaml"))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(here, "..", "resource", "waypoints.yaml"))

    for path in candidates:
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}, path
    return {}, None


def _apply_config():
    """로드한 yaml 값을 전역 변수에 반영."""
    global DEFAULT_SUBJECT_CENTER, WAYPOINTS
    global HOME_JOINT, VELOCITY, ACC, MOVE_TIME

    cfg, path = _load_waypoints()
    if path:
        node_.get_logger().info(f"waypoints.yaml 로드: {path}")
    else:
        node_.get_logger().warn(
            "waypoints.yaml 없음 - 코드 기본값 사용 (좌표 부정확 가능)")

    if cfg.get("subject_center"):
        DEFAULT_SUBJECT_CENTER = list(cfg["subject_center"])
    if cfg.get("waypoints"):
        WAYPOINTS = [list(wp) for wp in cfg["waypoints"]]
    if cfg.get("home_joint"):
        HOME_JOINT = list(cfg["home_joint"])

    motion = cfg.get("motion", {})
    VELOCITY = motion.get("velocity", 40)
    ACC = motion.get("acc", 40)
    MOVE_TIME = motion.get("move_time", 5.0)

    node_.get_logger().info(
        f"촬영 중심={DEFAULT_SUBJECT_CENTER} / 웨이포인트 {len(WAYPOINTS)}개")


def initialize_robot():
    global node_
    try:
        from DSR_ROBOT2 import set_tool, set_tcp
        set_tool("Tool Weight_2FG")
        set_tcp("2FG_TCP")
    except Exception as e:
        node_.get_logger().warn(f"로봇 초기화 경고 (드라이버 미연결): {e}")
    return True  # 실패해도 True 반환 → 액션 서버는 항상 띄움


def _trigger_capture(shot_idx):
    global capture_cli, capture_executor_, capture_node_, node_
    if capture_cli is None or capture_executor_ is None:
        node_.get_logger().error(f"[캡처{shot_idx}] capture_cli 없음")
        return False
    node_.get_logger().info(f"[캡처{shot_idx}] 서비스 대기 중...")
    if not capture_cli.wait_for_service(timeout_sec=2.0):
        node_.get_logger().warn(f"[캡처{shot_idx}] /capture_image 서비스 없음 - 스킵")
        return False
    node_.get_logger().info(f"[캡처{shot_idx}] 서비스 호출...")
    future = capture_cli.call_async(Trigger.Request())
    capture_executor_.spin_until_future_complete(future, timeout_sec=5.0)
    res = future.result()
    node_.get_logger().info(f"[캡처{shot_idx}] 결과: {res}")
    return bool(res and res.success)


def perform_task_loop():
    global task_running, trigger_event, node_
    global g_current_goal_handle, g_final_result, g_goal_request
    logger = node_.get_logger()

    while rclpy.ok():
        triggered = trigger_event.wait(timeout=1.0)
        if not rclpy.ok():
            break
        if not triggered:
            continue
        if task_running:
            trigger_event.clear()
            continue

        task_running = True
        start_time = time.time()
        try:
            # §11: DSR API 는 작업 스레드에서만 import/호출
            from DSR_ROBOT2 import (
                amovel, movej, check_motion, get_current_posx, DR_BASE,
            )
            from DR_common2 import posx, posj

            feedback_msg = PhotoSession.Feedback()

            def wait_with_feedback(msg, target_time, start_pct, end_pct, shot):
                # §18 스마트 피드백 패턴
                time.sleep(0.1)
                start_t = time.time()
                while check_motion() != 0:
                    elapsed = time.time() - start_t
                    ratio = min(elapsed / target_time, 1.0)
                    cur = start_pct + (end_pct - start_pct) * ratio
                    feedback_msg.feedback_string = msg
                    feedback_msg.progress_percentage = float(cur)
                    feedback_msg.current_shot = int(shot)
                    g_current_goal_handle.publish_feedback(feedback_msg)
                    time.sleep(0.1)
                    if not rclpy.ok():
                        return False
                feedback_msg.feedback_string = msg
                feedback_msg.progress_percentage = float(end_pct)
                feedback_msg.current_shot = int(shot)
                g_current_goal_handle.publish_feedback(feedback_msg)
                return True

            # Goal 파싱: 클라이언트가 subject_center 안 주면 yaml 값 사용
            req = g_goal_request
            if req and len(req.subject_center) == 6:
                center = list(req.subject_center)
            else:
                center = list(DEFAULT_SUBJECT_CENTER)

            # 촬영 포인트: yaml 의 절대좌표 waypoints 사용
            shot_list = WAYPOINTS if WAYPOINTS else [center]
            num_shots = req.num_shots if (req and req.num_shots > 0) \
                else len(shot_list)
            num_shots = min(num_shots, len(shot_list))

            JReady = posj(HOME_JOINT)
            movej(JReady, vel=60, acc=60)

            captured = 0
            for i in range(num_shots):
                wp = shot_list[i]
                target = posx([
                    wp[0], wp[1], wp[2], wp[3], wp[4], wp[5],
                ])
                seg = 90.0 / num_shots
                s_pct = i * seg
                e_pct = (i + 1) * seg

                # §18: amovel + time + check_motion 루프
                amovel(target, vel=VELOCITY, acc=ACC,
                       time=MOVE_TIME, ref=DR_BASE)
                ok = wait_with_feedback(
                    f"{i + 1}번 포인트 이동 중...",
                    MOVE_TIME, s_pct, e_pct, i + 1,
                )
                if not ok:
                    raise RuntimeError("모션 중단 (rclpy 종료)")

                time.sleep(0.3)        # 흔들림 안정화
                if _trigger_capture(i + 1):
                    captured += 1
                feedback_msg.feedback_string = f"{i + 1}컷 촬영 완료"
                feedback_msg.progress_percentage = float(e_pct)
                feedback_msg.current_shot = i + 1
                g_current_goal_handle.publish_feedback(feedback_msg)

            movej(JReady, vel=60, acc=60)

            result = PhotoSession.Result()
            result.complete_task = captured == num_shots
            result.captured_count = captured
            result.image_paths = []
            result.total_duration = time.time() - start_time
            cur_pose, _ = get_current_posx()
            result.final_pose = list(cur_pose)
            result.message = f"{captured}/{num_shots} 컷 촬영"
            g_final_result = result
            g_current_goal_handle.succeed()
            logger.info(result.message)

        except Exception as e:
            logger.error(f"촬영 작업 중 예외: {e}")
            result = PhotoSession.Result()
            result.complete_task = False
            result.captured_count = 0
            result.total_duration = time.time() - start_time
            result.message = str(e)
            g_final_result = result
            if g_current_goal_handle and g_current_goal_handle.is_active:
                g_current_goal_handle.abort()
        finally:
            task_running = False
            trigger_event.clear()
            g_current_goal_handle = None
            g_goal_request = None


def goal_callback(goal_request):
    global g_goal_request
    if task_running:
        return GoalResponse.REJECT
    if not goal_request.start_task:
        return GoalResponse.REJECT
    g_goal_request = goal_request
    return GoalResponse.ACCEPT


def execute_callback(goal_handle):
    global g_current_goal_handle, g_final_result
    g_current_goal_handle = goal_handle
    g_final_result = None
    trigger_event.set()
    while g_current_goal_handle is not None and rclpy.ok():
        time.sleep(0.1)
    return g_final_result if g_final_result else PhotoSession.Result()


def ros_spin_thread():
    global node_
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node_)
    _ = ActionServer(
        node_, PhotoSession, "do_photo_session",
        execute_callback=execute_callback,
        goal_callback=goal_callback,
        cancel_callback=None,
    )
    executor.spin()


def main(args=None):
    global node_, capture_node_, capture_cli, capture_executor_
    rclpy.init(args=args)

    # §주의: 여러 서버 동시 기동 시 드라이버 충돌 방지
    time.sleep(random.uniform(0.1, 1.0))

    node_ = rclpy.create_node(NODE_NAME, namespace=ROBOT_ID)
    DR_init.__dsr__node = node_     # §1

    # waypoints.yaml 로드
    _apply_config()

    # 캡처 서비스 전용 node — node_와 executor를 분리해 future 처리 보장
    capture_node_ = rclpy.create_node(NODE_NAME + "_capture_client")
    capture_cli = capture_node_.create_client(Trigger, "/capture_image")
    capture_executor_ = SingleThreadedExecutor()
    capture_executor_.add_node(capture_node_)

    initialize_robot()

    robot_thread = threading.Thread(target=perform_task_loop, daemon=True)
    spin_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    robot_thread.start()
    spin_thread.start()
    try:
        robot_thread.join()
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        if capture_executor_ is not None:
            capture_executor_.shutdown()
        if capture_node_ is not None:
            capture_node_.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()