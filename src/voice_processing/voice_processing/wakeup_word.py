"""
wakeup_word.py
2종 웨이크워드 감지 모듈.
  - 1st: "안녕 두봇"   -> 테마 주문 단계 진입
  - 2nd: "사진 찍어줘" -> 동적 촬영 모션 트리거

openWakeWord 모델이 있으면 사용하고, 없으면 STT 텍스트 매칭 폴백으로
동작하도록 이중화했다. (Day1~2 데모 안정성 확보용)
"""

import os
import numpy as np


# 웨이크워드 라벨 상수 (manager / get_keyword 에서 import 해 사용)
WAKEUP_HELLO = "hello_doobot"     # "안녕 두봇"
WAKEUP_SHOOT = "take_photo"       # "사진 찍어줘"

# STT 폴백 매칭용 한국어 트리거 문구
_HELLO_PHRASES = ["안녕 두봇", "안녕두봇", "두봇 안녕", "헬로 두봇"]
_SHOOT_PHRASES = ["사진 찍어", "사진찍어", "찍어줘", "촬영해", "사진 찍어줘"]


class WakeupWord:
    def __init__(self, buffer_size=24000, model_dir=None):
        self.buffer_size = buffer_size
        self.model = None
        self.model_name = None
        self.stream = None
        self._oww_available = False
        self._try_load_oww(model_dir)

    def _try_load_oww(self, model_dir):
        """openWakeWord 로드 시도. 실패 시 STT 폴백 모드."""
        try:
            from openwakeword.model import Model
            if model_dir and os.path.isdir(model_dir):
                paths = [
                    os.path.join(model_dir, f)
                    for f in os.listdir(model_dir)
                    if f.endswith((".onnx", ".tflite"))
                ]
                if paths:
                    self.model = Model(wakeword_models=paths)
                    self._oww_available = True
            if self.model is None:
                self.model = Model()          # 기본 사전학습 모델
                self._oww_available = True
        except Exception as e:
            print(f"[WakeupWord] openWakeWord 미사용 (STT 폴백): {e}")
            self._oww_available = False

    def set_stream(self, stream):
        self.stream = stream

    @property
    def available(self):
        return self._oww_available

    def is_wakeup(self, threshold=0.5):
        """
        오디오 스트림에서 웨이크워드 1회 감지 시도.
        반환: WAKEUP_HELLO / WAKEUP_SHOOT / None
        openWakeWord 미사용 환경에서는 항상 None -> get_keyword 가 STT 폴백 사용.
        """
        if not self._oww_available or self.stream is None:
            return None

        try:
            audio_chunk = np.frombuffer(
                self.stream.read(self.buffer_size, exception_on_overflow=False),
                dtype=np.int16,
            )
        except Exception:
            return None

        predictions = self.model.predict(audio_chunk)
        if not predictions:
            return None

        best_label = max(predictions, key=predictions.get)
        best_score = predictions[best_label]
        if best_score < threshold:
            return None

        label_low = best_label.lower()
        if "hello" in label_low or "hey" in label_low or "doobot" in label_low:
            return WAKEUP_HELLO
        if "photo" in label_low or "shoot" in label_low or "picture" in label_low:
            return WAKEUP_SHOOT
        return WAKEUP_HELLO     # 기본 단일모델이면 1st 로 간주

    @staticmethod
    def match_text(text: str):
        """
        STT 폴백: 인식된 문장에서 웨이크워드 종류 판별.
        반환: WAKEUP_HELLO / WAKEUP_SHOOT / None
        """
        if not text:
            return None
        t = text.replace(" ", "")
        for p in _SHOOT_PHRASES:
            if p.replace(" ", "") in t:
                return WAKEUP_SHOOT
        for p in _HELLO_PHRASES:
            if p.replace(" ", "") in t:
                return WAKEUP_HELLO
        return None
