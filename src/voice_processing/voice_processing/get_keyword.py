"""
get_keyword.py
GetTheme 서비스 서버 노드.
  1) 웨이크워드("안녕 두봇") 대기  -> 2) STT 녹음/인식
  -> 3) GPT-4o 로 테마 키워드 추출 -> 4) theme_map.yaml 로 소품 매핑
  -> 5) GetTheme.Response 로 반환

photo_booth_manager 가 클라이언트로 trigger=true 호출.
두산 코딩 규칙 §1 헤더 / §9 콜백(spin_once) 패턴 적용.
이 노드는 DSR API 를 호출하지 않으므로 DR_init 연결은 생략.
"""

import os
import json

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
    """resource/theme_map.yaml 로드. share -> 소스트리 순으로 폴백."""
    candidates = []
    try:
        share = get_package_share_directory("voice_processing")
        candidates.append(os.path.join(share, "resource", "theme_map.yaml"))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(here, "..", "resource", "theme_map.yaml"))

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
        self.get_logger().info(f"테마 맵 로드: {list(self.themes.keys())}")

        self.mic = MicController(MicConfig())
        self.wakeup = WakeupWord(buffer_size=MicConfig.buffer_size)
        _env_path = os.path.join(get_package_share_directory("voice_processing"), "resource", ".env")
        load_dotenv(dotenv_path=_env_path)
        self.stt = STT(record_seconds=5)

        # GPT 클라이언트 (키 없으면 키워드 매칭 폴백)
        self._gpt = None
        api_key = os.getenv("OPENAI_API_KEY")
        if api_key:
            try:
                from openai import OpenAI
                self._gpt = OpenAI(api_key=api_key)
            except Exception as e:
                self.get_logger().warn(f"GPT 비활성 (폴백 사용): {e}")

        self.srv = self.create_service(
            GetTheme, "get_theme", self.handle_get_theme
        )
        self.get_logger().info("GetTheme 서비스 서버 준비 완료 (/get_theme)")

    # ---- 웨이크워드 대기 ----
    def _wait_wakeup(self, timeout_sec=30.0):
        self.mic.open_stream()
        self.wakeup.set_stream(self.mic.stream)
        elapsed = 0.0
        step = 0.1
        while rclpy.ok() and elapsed < timeout_sec:
            if self.wakeup.available:
                if self.wakeup.is_wakeup() == WAKEUP_HELLO:
                    return True
            else:
                # openWakeWord 미사용 -> 즉시 STT 단계로 진행
                return True
            elapsed += step
        return False

    # ---- GPT 테마 추출 ----
    def _extract_theme(self, text: str) -> str:
        keys = list(self.themes.keys())
        if not text:
            return ""

        if self._gpt is not None:
            try:
                sys_prompt = (
                    "너는 사진부스 테마 분류기다. 사용자 발화를 다음 테마 중 "
                    f"하나로만 분류해 키워드만 출력해라. 후보: {keys}. "
                    "해당 없으면 'none' 출력."
                )
                resp = self._gpt.chat.completions.create(
                    model="gpt-4o",
                    messages=[
                        {"role": "system", "content": sys_prompt},
                        {"role": "user", "content": text},
                    ],
                    temperature=0.0,
                )
                cand = (resp.choices[0].message.content or "").strip().lower()
                cand = cand.replace("'", "").replace('"', "").strip()
                if cand in self.themes:
                    return cand
            except Exception as e:
                self.get_logger().warn(f"GPT 호출 실패, 폴백: {e}")

        # 폴백: theme_map 의 키워드 매칭
        low = text.lower()
        for name, info in self.themes.items():
            for kw in info.get("keywords", []):
                if kw.lower() in low:
                    return name
        return ""

    # ---- 서비스 콜백 ----
    def handle_get_theme(self, request, response):
        if not request.trigger:
            response.success = False
            response.message = "trigger=false"
            return response

        if not self._wait_wakeup():
            response.success = False
            response.message = "wakeup timeout"
            return response

        text = self.stt.speech2text(self.mic)
        response.raw_text = text

        theme = self._extract_theme(text)
        if not theme:
            response.success = False
            response.theme = ""
            response.props = []
            response.message = "theme not recognized"
            return response

        info = self.themes.get(theme, {})
        response.success = True
        response.theme = theme
        response.props = list(info.get("props", []))
        response.message = info.get("display_name", theme)
        self.get_logger().info(
            f"테마='{theme}' 소품={response.props}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GetKeywordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mic.close_stream()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
