"""
wakeup_word.py (Hello Rokey 커스텀 모델 적용 버전)
48kHz 오디오를 16kHz로 다운샘플링하여 openWakeWord 커스텀 모델을 구동합니다.
"""

import os
import numpy as np
from openwakeword.model import Model
from scipy.signal import resample
from ament_index_python.packages import get_package_share_directory

# 기존 시스템 호환성을 위해 리턴 상수 이름은 그대로 유지합니다.
WAKEUP_HELLO = "hello_doobot"     
WAKEUP_SHOOT = "take_photo"       

# STT 폴백 매칭용 한국어 트리거 문구 (혹시 모를 STT 대비용)
_HELLO_PHRASES = ["헬로 로키", "헬로로키", "헬로우 로키"]

class WakeupWord:
    def __init__(self, buffer_size=24000):
        self.buffer_size = buffer_size
        self.model = None
        # 모델명에서 확장자를 제외한 이름이 딕셔너리 키(key)가 됩니다.
        self.model_name = "hello_rokey_8332_32" 
        self.stream = None
        self._oww_available = False
        self._try_load_custom_model()

    def _try_load_custom_model(self):
        """fruit 프로젝트에서 가져온 커스텀 tflite 모델 로드"""
        try:
            share = get_package_share_directory("voice_processing")
            model_path = os.path.join(share, "resource", f"{self.model_name}.tflite")
            
            if os.path.exists(model_path):
                self.model = Model(wakeword_models=[model_path])
                self._oww_available = True
                print(f"[WakeupWord] 헬로 로키 모델 로드 성공: {model_path}", flush=True)
            else:
                print(f"[WakeupWord] 에러: 모델 파일을 찾을 수 없습니다 -> {model_path}", flush=True)
                
        except Exception as e:
            print(f"[WakeupWord] openWakeWord 초기화 실패: {e}", flush=True)
            self._oww_available = False

    def set_stream(self, stream):
        self.stream = stream

    @property
    def available(self):
        return self._oww_available

    def is_wakeup(self, threshold=0.3):
        """
        오디오 스트림에서 '헬로 로키' 1회 감지 시도.
        반환: WAKEUP_HELLO / None
        """
        if not self._oww_available or self.stream is None:
            return None

        try:
            audio_chunk = np.frombuffer(
                self.stream.read(self.buffer_size, exception_on_overflow=False),
                dtype=np.int16,
            )
            # [핵심] 48000Hz 마이크 입력을 모델이 좋아하는 16000Hz로 다운샘플링
            audio_chunk = resample(audio_chunk, int(len(audio_chunk) * 16000 / 48000))
            
        except Exception as e:
            print(f"[WakeupWord] 오디오 읽기 에러: {e}", flush=True)
            return None

        # 예측 수행
        predictions = self.model.predict(audio_chunk)
        if not predictions:
            return None

        # 커스텀 모델의 점수 확인
        confidence = predictions.get(self.model_name, 0.0)
        
        # 실시간 점수 출력 (테스트 시 값이 튀는지 확인 용도)
        print(f"Rokey Confidence: {confidence:.4f}", flush=True)

        if confidence > threshold:
            print("🎉 WakeWord Detected: 헬로 로키!", flush=True)
            # 시스템은 1번(안녕 두봇)이든 2번(사진 찍어줘)이든 모두 WAKEUP_HELLO 트리거로 통일해서 처리하게 됩니다.
            return WAKEUP_HELLO
            
        return None

    @staticmethod
    def match_text(text: str):
        """ STT 폴백 """
        if not text:
            return None
        t = text.replace(" ", "")
        for p in _HELLO_PHRASES:
            if p.replace(" ", "") in t:
                return WAKEUP_HELLO
        return None