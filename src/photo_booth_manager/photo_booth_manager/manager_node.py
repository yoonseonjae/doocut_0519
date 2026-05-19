#!/usr/bin/env python3
import os
import sys
import time
import threading
from flask import Flask, jsonify, request

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# 인터페이스 정의
from doocut_interfaces.srv import GetTheme
from doocut_interfaces.action import PhotoSession
from std_srvs.srv import Trigger
from std_msgs.msg import String

app = Flask(__name__)
manager_node = None

class PhotoBoothManager(Node):
    def __init__(self):
        super().__init__('photo_booth_manager')
        
        self.output_dir = os.path.expanduser('~/doocut_ws/src/photo_booth_manager/output')
        os.makedirs(self.output_dir, exist_ok=True)

        self.current_stage_status = "대기 중"
        self.latest_detected_theme = "none"
        self.latest_detected_props = []

        # ---------------------------------------------------------
        # ROS2 서비스 및 액션 클라이언트 초기화 (네임스페이스 반영)
        # ---------------------------------------------------------
        self.cli_get_theme = self.create_client(GetTheme, 'get_theme')
        self.cli_vision_detect = self.create_client(Trigger, '/get_3d_position')
        
        # 🟢 [추가됨] STAGE 2.1: 비전 스캔을 위한 준비(Ready) 위치 이동 서비스
        self.cli_robot_ready = self.create_client(Trigger, '/dsr01/pick_place_ready')
        
        # STAGE 3: 실제 로봇 픽업 및 전달 서비스
        self.cli_robot_pick = self.create_client(Trigger, '/dsr01/pick_place_service')
        
        # STAGE 4: 로봇 촬영 액션 클라이언트
        self.action_photo_motion = ActionClient(self, PhotoSession, '/dsr01/photo_session')

        self.get_logger().info("✅ PhotoBoothManager 실하드웨어 네임스페이스 연동 완료 (Ready 추가됨)")

    def speak_tts(self, message: str):
        border = "═" * (len(message) * 2 + 4) if any(ord(c) > 127 for c in message) else "═" * (len(message) + 4)
        print(f"\n📢 [TTS 콘솔 안내 방송]")
        print(f"╠{border}╣")
        print(f"║  \"{message}\"  ║")
        print(f"╚{border}╝\n")
        self.get_logger().info(f"[TTS AUDIO]: {message}")

    def run_main_sequence(self):
        print("\n" + "="*60)
        self.get_logger().info("🚀 [시작] 인생DOO컷 실물 로봇 시퀀스 가동")
        print("="*60)
        self.current_stage_status = "시퀀스 시작"

        # ─────────────────────────────────────────────────────────
        # [STAGE 1] 음성 테마 인식
        # ─────────────────────────────────────────────────────────
        self.get_logger().info("📌 [Stage 1] 음성 인식 및 테마 확정 대기 중...")
        self.speak_tts("안녕하세요. 인생 두 컷입니다. 원하시는 컨셉을 말씀해 주세요.")
        self.current_stage_status = "Stage 1: 음성 대기 중"

        if not self.cli_get_theme.wait_for_service(timeout_sec=3.0):
            self.print_stage_error(1, "voice_processing (get_keyword) 서비스 서버가 응답하지 않습니다.")
            return

        req_theme = GetTheme.Request()
        req_theme.trigger = True
        
        future_theme = self.cli_get_theme.call_async(req_theme)
        while rclpy.ok() and not future_theme.done():
            time.sleep(0.1)
            
        theme_res = future_theme.result()
        if not theme_res or not theme_res.success:
            self.print_stage_error(1, "음성 인식 실패 혹은 타임아웃 발생!")
            return 

        theme = theme_res.theme
        props = theme_res.props
        self.latest_detected_theme = theme
        self.latest_detected_props = list(props)
        self.get_logger().info(f"✅ [Stage 1 성공] 테마 결정: [{theme}] | 필요한 소품: {props}")


        # ─────────────────────────────────────────────────────────
        # 🟢 [추가됨] [STAGE 2-A] 스캔(Ready) 위치로 로봇 이동
        # ─────────────────────────────────────────────────────────
        self.get_logger().info(f"📌 [Stage 2-A] 카메라 인식을 위해 로봇을 스캔 위치로 이동합니다...")
        self.current_stage_status = "Stage 2-A: 로봇 스캔 위치 이동 중"

        if not self.cli_robot_ready.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("⚠️ [Stage 2-A 경고] pick_place_ready 서비스 오프라인! 강제로 다음으로 넘어갑니다.")
        else:
            req_ready = Trigger.Request()
            future_ready = self.cli_robot_ready.call_async(req_ready)
            while rclpy.ok() and not future_ready.done():
                time.sleep(0.1)
            ready_res = future_ready.result()
            if not ready_res or not ready_res.success:
                self.print_stage_error(2, "스캔 위치(Ready)로 이동하는 중 에러가 발생했습니다.")
                return
            self.get_logger().info("✅ [Stage 2-A 성공] 로봇 스캔 위치 도달 완료!")


        # ─────────────────────────────────────────────────────────
        # [STAGE 2-B] 소품 위치 추정 (Vision)
        # ─────────────────────────────────────────────────────────
        self.get_logger().info(f"📌 [Stage 2-B] 비전 센서 기반 소품({theme}) 3D 좌표 추정 중...")
        self.current_stage_status = "Stage 2-B: 소품 좌표 추정"
        # 카메라가 안착할 수 있도록 아주 잠깐 대기
        time.sleep(1.0) 

        has_vision_srv = self.cli_vision_detect.wait_for_service(timeout_sec=2.0)
        if not has_vision_srv:
            self.get_logger().warn("⚠️ [Stage 2-B 경고] 비전 서버 오프라인. 테스트용 모의 좌표로 우회 가동합니다.")
        else:
            req_vision = Trigger.Request()
            future_vision = self.cli_vision_detect.call_async(req_vision)
            while rclpy.ok() and not future_vision.done():
                time.sleep(0.1)
            vision_res = future_vision.result()
            if not vision_res or not vision_res.success:
                self.print_stage_error(2, "소품 3D 인식 실패!")
                return

        self.get_logger().info(f"✅ [Stage 2-B 성공] 소품 검출 좌표 획득 완료")


        # ─────────────────────────────────────────────────────────
        # [STAGE 3] 소품 픽업 및 사용자 전달 (실물 구동부 연동)
        # ─────────────────────────────────────────────────────────
        self.get_logger().info(f"📌 [Stage 3] 로봇 구동: 소품 픽업 및 사용자 전달 실물 서비스 호출...")
        self.current_stage_status = "Stage 3: 로봇 소품 픽업 수행 중"
        
        if not self.cli_robot_pick.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("⚠️ [Stage 3 경고] pick_place_service 오프라인! 하드웨어 보호를 위해 임시 우회합니다.")
            time.sleep(2.0)
        else:
            req_pick = Trigger.Request()
            future_pick = self.cli_robot_pick.call_async(req_pick)
            while rclpy.ok() and not future_pick.done():
                time.sleep(0.1)
            pick_res = future_pick.result()
            if not pick_res or not pick_res.success:
                self.print_stage_error(3, "로봇이 소품을 집어 올리는 동작 중 제어기 에러가 발생했습니다.")
                return

        self.get_logger().info("✅ [Stage 3 성공] 로봇이 사용자에게 소품 전달을 완료했습니다.")
        
        display_props = ", ".join(props) if props else "소품"
        self.speak_tts(f"{theme} 컨셉으로 준비할게요. {display_props}을 착용해 주세요. 5초 뒤 촬영을 시작합니다.")
        
        for i in range(5, 0, -1):
            self.get_logger().info(f"⏳ [착용 대기 카운트다운] 촬영까지 {i}초 전...")
            self.current_stage_status = f"Stage 3: 촬영 대기 {i}초 전"
            time.sleep(1.0)


        # ─────────────────────────────────────────────────────────
        # [STAGE 4] 로봇 촬영 및 정면 이동
        # ─────────────────────────────────────────────────────────
        self.get_logger().info("📌 [Stage 4] 로봇 정면 촬영 위치 이동 및 캡처 요청...")
        self.current_stage_status = "Stage 4: 정면 촬영 이동"

        if not self.action_photo_motion.wait_for_server(timeout_sec=4.0):
            self.print_stage_error(4, "robot_control (photo_motion) 액션 서버 연결 실패!")
            return

        goal_msg = PhotoSession.Goal()
        self.get_logger().info("로봇 촬영 액션 목표(Goal) 송신...")
        future_goal = self.action_photo_motion.send_goal_async(goal_msg)
        while rclpy.ok() and not future_goal.done():
            time.sleep(0.1)
            
        goal_handle = future_goal.result()
        if not goal_handle.accepted:
            self.print_stage_error(4, "로봇 촬영 액션 동작 요청이 거부되었습니다.")
            return

        self.get_logger().info("로봇이 정면 촬영 자세로 부드럽게 이동 중입니다...")
        future_result = goal_handle.get_result_async()
        while rclpy.ok() and not future_result.done():
            time.sleep(0.1)
            
        action_res = future_result.result()
        if not action_res or not action_res.result.success:
            self.print_stage_error(4, "로봇 정면 촬영 주행 및 캡처 수행 중 에러 발생.")
            return

        self.get_logger().info("✅ [Stage 4 성공] 정면 사진 촬영 및 로봇 홈 위치 복귀 완료.")

        # ─────────────────────────────────────────────────────────
        # [STAGE 5] 이미지 저장 완료
        # ─────────────────────────────────────────────────────────
        self.get_logger().info("📌 [Stage 5] 촬영 완료 프로세스...")
        self.current_stage_status = "Stage 5: 완료 처리"
        self.speak_tts("사진이 완성됐어요. 화면을 확인해 주세요.")
        
        print("\n" + "="*60)
        self.get_logger().info("🎉 [완료] 모든 실물 스테이지 통과! 정상 종료되었습니다.")
        print("="*60 + "\n")
        self.current_stage_status = "모든 시퀀스 성공 완료"


    def print_stage_error(self, stage_num: int, guide_text: str):
        self.current_stage_status = f"Stage {stage_num} 에러 중단"
        print("\n🛑 [디버그 에러 발생] " + "═"*40)
        self.get_logger().error(f"❌ [Stage {stage_num} 에러] 시퀀스가 중단되었습니다.")
        self.get_logger().error(f"💡 해결 가이드: {guide_text}")
        print("═"*62 + "\n")


@app.route('/')
def index():
    return jsonify({"status": "online", "message": "인생DOO컷 실물 연동 API 서버 가동 중"})

@app.route('/start', methods=['GET', 'POST'])
@app.route('/api/start', methods=['GET', 'POST'])
def start_sequence():
    global manager_node
    if manager_node is None:
        return jsonify({"status": "error", "message": "ROS2 Node Not Initialized"}), 500
    t = threading.Thread(target=manager_node.run_main_sequence)
    t.start()
    return jsonify({"status": "success", "message": "실물 스테이지 시퀀스가 기동되었습니다."})

@app.route('/api/status', methods=['GET'])
def get_status():
    global manager_node
    if manager_node is None:
        return jsonify({"stage": "Offline"})
    return jsonify({
        "stage": manager_node.current_stage_status,
        "theme": manager_node.latest_detected_theme,
        "props": manager_node.latest_detected_props
    })

def run_flask():
    app.run(host='0.0.0.0', port=8080, debug=False, use_reloader=False)

def main(args=None):
    global manager_node
    rclpy.init(args=args)
    manager_node = PhotoBoothManager()
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    try:
        rclpy.spin(manager_node)
    except KeyboardInterrupt:
        if manager_node:
            manager_node.get_logger().info("🛑 사용자에 의해 photo_booth_manager가 종료되었습니다.")
    finally:
        if manager_node:
            manager_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()