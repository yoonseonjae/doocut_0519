"""
get_keyword.py (단일 인식 후 종료 버전)
  - 노드 실행 시 상시 대기하지 않고, 서비스 요청이 올 때만 마이크를 엽니다.
  - "헬로 로키"가 한 번 인식되면 STT/GPT 분석 후 즉시 결과를 반환하고 마이크를 닫습니다.
"""

import os
import time
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from doocut_interfaces.srv import GetTheme

from voice_processing.MicController import MicController, MicConfig
from voice_processing.wakeup_word import WakeupWord, WAKEUP_HELLO
from voice_processing.STT import STT
from dotenv import load_dotenv

NODE_NAME = "get_keyword_node"

def _load_theme_map():
    candidates = []
    try:
        share = get_package_share_directory("voice_processing")
        candidates.append(os.path.join(share, "resource", "theme_map.yaml"))
    except Exception:
        pass
    for path in candidates:
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
    return {}

class GetKeywordNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.theme_map = _load_theme_map()
        self.themes = self.theme_map.get("themes", {})
        
        self.mic = MicController(MicConfig())
        self.wakeup = WakeupWord(buffer_size=MicConfig.buffer_size)
        
        _env_path = os.path.join(get_package_share_directory("voice_processing"), "resource", ".env")
        load_dotenv(dotenv_path=_env_path)
        self.stt = STT(record_seconds=5)

        self._gpt = None
        api_key = os.getenv("OPENAI_API_KEY")
        if api_key:
            try:
                from openai import OpenAI
                self._gpt = OpenAI(api_key=api_key)
            except Exception as e:
                self.get_logger().warn(f"GPT 비활성: {e}")

        self.srv = self.create_service(GetTheme, "get_theme", self.handle_get_theme)
        self.get_logger().info("GetTheme 서버 준비 완료 (단일 인식 모드)")

    def _extract_theme(self, text: str) -> str:
        keys = list(self.themes.keys())
        if not text or self._gpt is None:
            return ""
        try:
            sys_prompt = f"사진부스 테마 분류기다. 후보: {keys}. 키워드 하나만 출력해라. 없으면 'none'."
            resp = self._gpt.chat.completions.create(
                model="gpt-4o",
                messages=[{"role": "system", "content": sys_prompt}, {"role": "user", "content": text}],
                temperature=0.0,
            )
            cand = resp.choices[0].message.content.strip().lower()
            return cand if cand in self.themes else ""
        except Exception:
            return ""

    def handle_get_theme(self, request, response):
        self.get_logger().info("🎙️ '헬로 로키' 대기를 시작합니다...")
        
        # 1. 마이크 오픈 및 스트림 연결
        self.mic.open_stream()
        self.wakeup.set_stream(self.mic.stream)
        
        detected = False
        start_wait = time.time()
        timeout = 30.0 # 30초 동안 인식 없으면 종료

        # 2. 웨이크워드 감지 루프 (인식될 때까지만 수행)
        while rclpy.ok():
            if (time.time() - start_wait) > timeout:
                break

            if self.wakeup.is_wakeup(threshold=0.3) == WAKEUP_HELLO:
                self.get_logger().info("🎯 감지 성공! 분석 시작.")
                detected = True
                break # 🔴 웨이크워드 인식 즉시 루프 탈출 (한 번만 수행)
            
            time.sleep(0.01)

        if not detected:
            response.success = False
            response.message = "인식 실패 또는 타임아웃"
            self.mic.close_stream()
            return response

        # 3. 음성 분석 (STT -> GPT)
        text = self.stt.speech2text(self.mic)
        theme = self._extract_theme(text)
        info = self.themes.get(theme, {})

        # 4. 결과 설정
        response.success = True if theme else False
        response.theme = theme
        response.props = list(info.get("props", []))
        response.message = info.get("display_name", theme)
        
        # 5. 마이크 닫기 (작업 종료)
        self.mic.close_stream()
        self.get_logger().info(f"✅ 분석 완료: {theme}. 마이크를 닫습니다.")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GetKeywordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()