"""
tts_node.py
안내 방송 TTS 노드.

  - /tts (std_msgs/String) 구독 -> 큐에 적재 -> 워커 스레드가 순차 재생
  - 백엔드 우선순위: gTTS(온라인) -> pyttsx3(오프라인) -> 콘솔 출력
  - 동시 발화 충돌 방지 위해 단일 워커 스레드 + Queue

DSR API 미사용 노드 (DR_init 연결 불필요).
"""

import os
import queue
import tempfile
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

NODE_NAME = "tts_node"


class TTSBackend:
    """gTTS -> pyttsx3 -> print 순으로 가용 백엔드 선택."""

    def __init__(self, logger):
        self.logger = logger
        self.mode = "print"
        self._pyttsx_engine = None
        self._select()

    def _select(self):
        try:
            import gtts  # noqa: F401
            self.mode = "gtts"
            self.logger.info("TTS 백엔드: gTTS")
            return
        except Exception:
            pass
        try:
            import pyttsx3
            self._pyttsx_engine = pyttsx3.init()
            self.mode = "pyttsx3"
            self.logger.info("TTS 백엔드: pyttsx3 (오프라인)")
            return
        except Exception:
            pass
        self.logger.warn("TTS 백엔드 없음 - 콘솔 출력 폴백")

    def _play_file(self, path):
        """mp3/wav 재생. mpg123 -> ffplay -> playsound 순 시도."""
        for cmd in (f'mpg123 -q "{path}"', f'ffplay -nodisp -autoexit -loglevel quiet "{path}"'):
            if os.system(cmd) == 0:
                return True
        try:
            from playsound import playsound
            playsound(path)
            return True
        except Exception:
            return False

    def speak(self, text: str):
        if not text:
            return
        if self.mode == "gtts":
            try:
                from gtts import gTTS
                tmp = tempfile.NamedTemporaryFile(
                    suffix=".mp3", delete=False)
                tmp_path = tmp.name
                tmp.close()
                gTTS(text=text, lang="ko").save(tmp_path)
                ok = self._play_file(tmp_path)
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass
                if ok:
                    return
            except Exception as e:
                self.logger.warn(f"gTTS 실패, 폴백: {e}")

        if self.mode == "pyttsx3" or self._pyttsx_engine is not None:
            try:
                if self._pyttsx_engine is None:
                    import pyttsx3
                    self._pyttsx_engine = pyttsx3.init()
                self._pyttsx_engine.say(text)
                self._pyttsx_engine.runAndWait()
                return
            except Exception as e:
                self.logger.warn(f"pyttsx3 실패, 폴백: {e}")

        self.logger.info(f"[TTS-콘솔] {text}")


class TTSNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.backend = TTSBackend(self.get_logger())
        self._q = queue.Queue()
        self._stop = threading.Event()

        self.sub = self.create_subscription(
            String, "/tts", self._on_msg, 10
        )
        self._worker = threading.Thread(
            target=self._worker_loop, daemon=True)
        self._worker.start()
        self.get_logger().info("TTS 노드 준비 완료 (/tts 구독)")

    def _on_msg(self, msg: String):
        self._q.put(msg.data)

    def _worker_loop(self):
        while not self._stop.is_set() and rclpy.ok():
            try:
                text = self._q.get(timeout=0.5)
            except queue.Empty:
                continue
            self.backend.speak(text)
            self._q.task_done()

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
