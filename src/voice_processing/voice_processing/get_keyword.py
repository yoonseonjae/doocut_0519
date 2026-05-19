# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"
# 응답 형식: "<scene> <prop1> <prop2> ..."  예) "beach umbrella bucket starfish"

import os
import rclpy
import pyaudio
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI

from std_srvs.srv import Trigger
from voice_processing.MicController import MicController, MicConfig
from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT
from voice_processing.scene_keyword_extractor import SceneKeywordExtractor

############ Package Path & Environment Setting ############

#----------------------------------------------------------------
# current_dir = os.getcwd()
# package_path = get_package_share_directory("pick_and_place_voice")

# env_path = "/home/rokey/cobot_ws/src/cobot2_ws/pick_and_place_voice/resource/.env"
# load_dotenv(dotenv_path=env_path)
# is_load = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
# openai_api_key = os.getenv("OPENAI_API_KEY")
#-----------------------------------------------------------------

PACKAGE_NAME = "voice_processing"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
ENV_PATH = os.path.join(RESOURCE_PATH, ".env")
load_dotenv(dotenv_path=ENV_PATH)
openai_api_key = os.getenv("OPENAI_API_KEY")

############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):

        print(PACKAGE_PATH, RESOURCE_PATH, ENV_PATH)

        # LLM: scene_keyword_extractor의 fallback용으로만 사용
        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.2, openai_api_key=openai_api_key
        )
        self.scene_extractor = SceneKeywordExtractor(llm=self.llm)
        self.stt = STT(openai_api_key=openai_api_key)

        super().__init__("get_keyword_node")
        # 오디오 설정 (기존 그대로)
        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)

        self.get_logger().info("GetKeyword (scene mode) initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

    def extract_keyword(self, utterance: str) -> list[str]:
        """
        STT 결과에서 scene을 추출하고 해당 소품 목록을 반환합니다.
        반환: [scene, prop1, prop2, ...] 형태의 리스트
        """
        scene = self.scene_extractor.extract_scene(utterance)
        if scene is None:
            self.get_logger().warn(f"No scene matched for utterance: '{utterance}'")
            return []

        props = self.scene_extractor.get_props(scene)
        self.get_logger().info(f"Scene: '{scene}' → props: {props}")
        return [scene] + props
    
    def get_keyword(self, request, response):
        """서비스 핸들러. 응답 형식: '<scene> <prop1> <prop2> ...'"""
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            response.success = False
            response.message = "audio_error"
            return response

        # wake word 감지 대기 (기존 wakeup_word.py 그대로)
        while not self.wakeup_word.is_wakeup():
            pass

        # STT → Scene 추출 → Props 반환
        utterance = self.stt.speech2text()
        self.get_logger().info(f"STT result: '{utterance}'")

        result = self.extract_keyword(utterance)  # [scene, prop1, prop2, ...]

        if not result:
            self.get_logger().warn("Failed to extract scene from utterance.")
            response.success = False
            response.message = "no_scene"
            return response

        self.get_logger().warn(f"Scene + Props: {result}")
        response.success = True
        response.message = " ".join(result)  # "beach umbrella bucket starfish"
        return response


def main():  # d2 메인문 일부 수정
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
