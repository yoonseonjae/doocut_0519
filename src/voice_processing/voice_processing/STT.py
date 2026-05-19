"""
STT.py
OpenAI Whisper API 기반 음성 -> 텍스트 변환 모듈.
.env 의 OPENAI_API_KEY 를 사용한다. (키 파일은 사용자가 직접 작성)
"""

import os
import wave
import tempfile

import pyaudio


class STT:
    def __init__(self, openai_api_key: str = None, record_seconds: int = 5):
        self.record_seconds = record_seconds
        self.rate = 48000
        self.channels = 1
        self.chunk = 12000
        self.fmt = pyaudio.paInt16

        api_key = openai_api_key or os.getenv("OPENAI_API_KEY")
        self._client = None
        if api_key:
            try:
                from openai import OpenAI
                self._client = OpenAI(api_key=api_key)
            except Exception as e:
                print(f"[STT] OpenAI 클라이언트 초기화 실패: {e}")
        else:
            print("[STT] OPENAI_API_KEY 미설정 - speech2text 비활성")

    def _record_to_wav(self, mic_controller) -> str:
        """마이크에서 record_seconds 만큼 녹음 후 임시 wav 경로 반환."""
        stream = mic_controller.open_stream()
        frames = []
        loops = int(self.rate / self.chunk * self.record_seconds)
        for _ in range(max(1, loops)):
            frames.append(stream.read(self.chunk, exception_on_overflow=False))

        tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
        tmp_path = tmp.name
        tmp.close()

        wf = wave.open(tmp_path, "wb")
        wf.setnchannels(self.channels)
        wf.setsampwidth(pyaudio.get_sample_size(self.fmt))
        wf.setframerate(self.rate)
        wf.writeframes(b"".join(frames))
        wf.close()
        return tmp_path

    def speech2text(self, mic_controller) -> str:
        """녹음 -> Whisper 변환 -> 텍스트 반환. 실패 시 빈 문자열."""
        if self._client is None:
            return ""

        wav_path = None
        try:
            wav_path = self._record_to_wav(mic_controller)
            with open(wav_path, "rb") as f:
                result = self._client.audio.transcriptions.create(
                    model="whisper-1",
                    file=f,
                    language="ko",
                )
            text = (result.text or "").strip()
            print(f"[STT] 인식 결과: {text}")
            return text
        except Exception as e:
            print(f"[STT] 변환 실패: {e}")
            return ""
        finally:
            if wav_path and os.path.exists(wav_path):
                try:
                    os.remove(wav_path)
                except OSError:
                    pass
