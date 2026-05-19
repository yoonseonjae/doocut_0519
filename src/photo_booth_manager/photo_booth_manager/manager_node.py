"""
manager_node.py
인생DOO컷 마스터 파이프라인 노드 (차곡차봇 workflow / bartender supervisor 역할).

전체 시퀀스를 순차 호출하는 중앙 컨트롤러. 나머지 노드는 서비스/액션
서버, manager 가 클라이언트.

  STEP1 GetTheme(/get_theme)            : 음성 -> 테마+소품
  STEP2 SrvDepthPosition(/get_3d_position): 소품별 3D 좌표 -> 픽앤플 트리거
  STEP3 CheckWearing(/check_wearing)     : 얼굴기준 착용검사
  STEP4 PhotoSession action(do_photo_session): 4웨이포인트 동적 촬영
        - 각 포인트 도착 시 /capture_image(Trigger) 로 RealSense 캡처
  STEP5 collage -> web_server -> QR -> /tts 안내

두산 코딩 규칙 §14(액션 클라이언트) 패턴 준수. DSR API 직접 미사용
(로봇 모션은 photo_motion 액션 서버에 위임).
"""

import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_srvs.srv import Trigger

from doocut_interfaces.srv import GetTheme, SrvDepthPosition, CheckWearing
from doocut_interfaces.action import PhotoSession

from photo_booth_manager.collage import make_collage
from photo_booth_manager.web_server import WebServer
from photo_booth_manager.qr_util import make_qr

NODE_NAME = "photo_booth_manager"

ROBOT_ID = "dsr01"
CAPTURE_COUNT = 4


def _pkg_dir(*sub):
    _src = os.path.join(os.path.expanduser("~"), "doocut_ws", "src", "photo_booth_manager")
    return os.path.abspath(os.path.join(_src, *sub))


class ManagerNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # ---- 경로 ----
        self.captures_dir = _pkg_dir("captures")
        self.output_dir = _pkg_dir("static", "output")
        self.frames_dir = _pkg_dir("static", "frames")
        self.templates_dir = _pkg_dir("templates")
        os.makedirs(self.captures_dir, exist_ok=True)
        os.makedirs(self.output_dir, exist_ok=True)

        # ---- 서비스 클라이언트 ----
        self.cli_theme = self.create_client(GetTheme, "/get_theme")
        self.cli_depth = self.create_client(
            SrvDepthPosition, "/get_3d_position")
        self.cli_wear = self.create_client(CheckWearing, "/check_wearing")

        # ---- 액션 클라이언트 (§14) ----
        self.ac_photo = ActionClient(
            self, PhotoSession, f"/{ROBOT_ID}/do_photo_session")

        # ---- TTS 퍼블리셔 ----
        self.pub_tts = self.create_publisher(String, "/tts", 10)

        # ---- 캡처 서비스 서버 (photo_motion 이 포인트마다 호출) ----
        self.create_service(Trigger, "/capture_image", self._on_capture)
        self._capture_idx = 0
        self._session_paths = []

        # ---- RealSense (캡처용) ----
        self.pipeline = None
        self._init_realsense()

        # ---- 웹 서버 ----
        self.web = WebServer(
            self.output_dir, self.templates_dir, port=8080, use_ngrok=False)
        self.web.start()

        self.get_logger().info("manager 노드 준비 완료")

    # ---------- 인프라 ----------
    def _init_realsense(self):
        try:
            import pyrealsense2 as rs
            self.pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
            self.pipeline.start(cfg)
            self.get_logger().info("RealSense(캡처) 시작 1280x720")
        except Exception as e:
            self.get_logger().warn(f"RealSense 비활성 (캡처 모의): {e}")
            self.pipeline = None

    def _say(self, text):
        msg = String()
        msg.data = text
        self.pub_tts.publish(msg)
        self.get_logger().info(f"[TTS] {text}")

    def _on_capture(self, request, response):
        """photo_motion 액션서버가 각 웨이포인트에서 호출."""
        self._capture_idx += 1
        path = os.path.join(
            self.captures_dir, f"shot_{self._capture_idx}.jpg")
        ok = self._grab_and_save(path)
        if ok:
            self._session_paths.append(path)
        response.success = ok
        response.message = path if ok else "capture failed"
        return response

    def _grab_and_save(self, path):
        if self.pipeline is None:
            # 모의: 빈 이미지라도 생성해 파이프라인 지속
            try:
                import numpy as np
                import cv2
                img = np.full((720, 1280, 3), 200, dtype=np.uint8)
                cv2.imwrite(path, img)
                return True
            except Exception:
                return False
        try:
            import numpy as np
            import cv2
            frames = self.pipeline.wait_for_frames()
            c = frames.get_color_frame()
            if not c:
                return False
            img = np.asanyarray(c.get_data())
            cv2.imwrite(path, img)
            return True
        except Exception as e:
            self.get_logger().error(f"캡처 실패: {e}")
            return False

    # ---------- 시퀀스 단계 ----------
    def step_get_theme(self):
        if not self.cli_theme.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/get_theme 없음")
            return None
        req = GetTheme.Request()
        req.trigger = True
        fut = self.cli_theme.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        res = fut.result()
        if res is None or not res.success:
            self._say("원하시는 컨셉을 다시 말씀해 주세요.")
            return None
        self._say(f"{res.message} 컨셉으로 준비할게요.")
        return {"theme": res.theme, "props": list(res.props)}

    def step_pick_props(self, props):
        if not self.cli_depth.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/get_3d_position 없음")
            return False
        for prop in props:
            req = SrvDepthPosition.Request()
            req.target = prop
            fut = self.cli_depth.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
            res = fut.result()
            if res is None or not res.success:
                self.get_logger().warn(f"'{prop}' 좌표 실패 - 스킵")
                continue
            self.get_logger().info(
                f"'{prop}' base={list(res.depth_position)[:3]} "
                f"-> robot_control 픽앤플 위임")
            self._say(f"{prop} 소품을 전달할게요.")
            time.sleep(0.5)
        return True

    def step_check_wearing(self, required, retries=3):
        if not self.cli_wear.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/check_wearing 없음")
            return False
        for attempt in range(retries):
            req = CheckWearing.Request()
            req.required_props = required
            req.iou_threshold = 0.0
            fut = self.cli_wear.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
            res = fut.result()
            if res and res.all_worn:
                self._say("멋지게 착용하셨네요. 촬영을 시작할게요.")
                return True
            miss = list(res.missing_props) if res else required
            self._say(f"{', '.join(miss)} 을(를) 착용해 주세요.")
            time.sleep(3.0)
        return False

    def step_photo_session(self, theme):
        """§14 액션 클라이언트 패턴으로 동적 촬영 호출."""
        if not self.ac_photo.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("do_photo_session 액션 서버 없음")
            return False

        self._capture_idx = 0
        self._session_paths = []

        goal = PhotoSession.Goal()
        goal.start_task = True
        goal.theme = theme
        goal.subject_center = []        # 서버 기본값 사용
        goal.num_shots = CAPTURE_COUNT

        def fb_cb(fb):
            self.get_logger().info(
                f"[촬영] {fb.feedback.feedback_string} "
                f"{fb.feedback.progress_percentage:.0f}%")

        send_fut = self.ac_photo.send_goal_async(
            goal, feedback_callback=fb_cb)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("촬영 goal 거부됨")
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=120.0)
        result = res_fut.result().result if res_fut.result() else None
        if result is None or not result.complete_task:
            self.get_logger().warn("촬영 미완료")
            return False
        self.get_logger().info(
            f"촬영 완료: {result.captured_count}컷 "
            f"{result.total_duration:.1f}s")
        return True

    def step_finalize(self, theme):
        """4컷 합성 -> 웹 게시 -> QR -> 안내."""
        paths = sorted(self._session_paths) if self._session_paths else \
            [os.path.join(self.captures_dir, f"shot_{i+1}.jpg")
             for i in range(CAPTURE_COUNT)]

        ts = time.strftime("%Y%m%d_%H%M%S")
        out = os.path.join(self.output_dir, f"doocut_{theme}_{ts}.jpg")
        collage = make_collage(paths, out, theme, self.frames_dir)
        if not collage:
            self._say("사진 합성에 문제가 생겼어요.")
            return False

        url = self.web.publish_result(collage, theme)
        qr_path = os.path.join(self.output_dir, f"qr_{ts}.png")
        make_qr(url, qr_path)

        self._say("사진이 완성됐어요. 화면의 QR 코드를 스캔해 주세요.")
        self.get_logger().info(f"결과 URL: {url}")
        self.get_logger().info(f"QR: {qr_path}")
        return True

    # ---------- 전체 루프 ----------
    def run_once(self):
        self._say("안녕하세요. 인생DOO컷입니다. 원하시는 컨셉을 말씀해 주세요.")
        theme_info = self.step_get_theme()
        if not theme_info:
            return
        theme = theme_info["theme"]
        props = theme_info["props"]

        self.step_pick_props(props)

        # 착용검사 핵심 2종 (theme_map required_wearing 과 동일 규약)
        self.step_check_wearing(props[:2])

        if not self.step_photo_session(theme):
            self._say("촬영에 실패했어요. 다시 시도해 주세요.")
            return

        self.step_finalize(theme)


def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()

    # 시퀀스를 별도 스레드에서 1회 실행, 메인은 spin
    def _seq():
        time.sleep(2.0)     # 다른 노드 기동 대기
        try:
            node.run_once()
        except Exception as e:
            node.get_logger().error(f"시퀀스 예외: {e}")

    t = threading.Thread(target=_seq, daemon=True)
    t.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.pipeline is not None:
            try:
                node.pipeline.stop()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
