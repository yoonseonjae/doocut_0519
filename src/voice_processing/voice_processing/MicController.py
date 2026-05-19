"""
MicController.py
마이크 입력 스트림 제어 모듈. WakeupWord / STT 가 공유한다.
ROS2 노드가 아니라 순수 오디오 유틸이므로 두산 코딩 규칙 영향 없음.
"""

import sys
import pyaudio


class MicConfig:
    chunk = 12000
    rate = 48000
    channels = 1
    record_seconds = 5
    fmt = pyaudio.paInt16
    device_index = None          # None 이면 시스템 기본 입력 장치
    buffer_size = 24000


class MicController:
    def __init__(self, config: MicConfig = None):
        self.config = config if config is not None else MicConfig()
        self.audio = None
        self.stream = None

    def open_stream(self):
        """입력 스트림을 연다. 이미 열려있으면 재사용."""
        if self.stream is not None and self.stream.is_active():
            return self.stream

        self.audio = pyaudio.PyAudio()
        try:
            self.stream = self.audio.open(
                format=self.config.fmt,
                channels=self.config.channels,
                rate=self.config.rate,
                input=True,
                input_device_index=self.config.device_index,
                frames_per_buffer=self.config.chunk,
            )
        except Exception as e:
            print(f"[MicController] 스트림 오픈 실패: {e}", file=sys.stderr)
            self.close_stream()
            raise
        return self.stream

    def read(self, num_frames=None):
        """오디오 프레임을 읽어 bytes 로 반환."""
        if self.stream is None:
            self.open_stream()
        n = num_frames if num_frames is not None else self.config.chunk
        return self.stream.read(n, exception_on_overflow=False)

    def close_stream(self):
        """스트림과 PyAudio 자원을 정리."""
        try:
            if self.stream is not None:
                if self.stream.is_active():
                    self.stream.stop_stream()
                self.stream.close()
        except Exception:
            pass
        finally:
            self.stream = None

        try:
            if self.audio is not None:
                self.audio.terminate()
        except Exception:
            pass
        finally:
            self.audio = None

    def __del__(self):
        self.close_stream()
