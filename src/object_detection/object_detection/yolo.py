"""
yolo.py
Ultralytics YOLO 래퍼.
  - 4단계 fallback model resolver (cobot2 model_paths.py 방식)
  - 다중프레임 탐지 + IoU 집계 유틸 (착용검사/안정 탐지 공용)

best.pt 자체는 사람이 라벨링/학습으로 생성해야 하지만, 그 파일을
"어디서 찾을지" 결정하는 resolver 와 추론 코드는 완전 동작한다.
"""

import os
import json

import numpy as np


# --------- 4단계 fallback resolver (cobot2 방식) ---------
def resolve_model_path(param_path: str = None,
                        pkg_name: str = "object_detection",
                        filename: str = "best.pt") -> str:
    """
    ① 파라미터 절대경로 -> ② cwd 상대 -> ③ ament-share resource/
    -> ④ 소스트리 resource/
    """
    cands = []
    if param_path:
        cands.append(param_path)
    cands.append(os.path.join(os.getcwd(), filename))
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(pkg_name)
        cands.append(os.path.join(share, "resource", filename))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    cands.append(os.path.join(here, "..", "resource", filename))

    for c in cands:
        if c and os.path.exists(c):
            return os.path.abspath(c)
    # 못 찾아도 마지막 후보 경로를 반환 (에러 메시지에 경로 노출용)
    return os.path.abspath(cands[-1])


def resolve_class_map(pkg_name: str = "object_detection",
                      filename: str = "class_name.json") -> dict:
    path = resolve_model_path(None, pkg_name, filename)
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}
    return {}


# --------- IoU 유틸 ---------
def iou_xyxy(box_a, box_b) -> float:
    """[x1,y1,x2,y2] 두 박스의 IoU."""
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return float(inter / union) if union > 0 else 0.0


class YoloDetector:
    def __init__(self, model_path: str = None, conf: float = 0.5):
        self.conf = conf
        self.model = None
        self.names = {}
        self.model_path = resolve_model_path(model_path)
        self._load()

    def _load(self):
        if not os.path.exists(self.model_path):
            print(f"[YoloDetector] 모델 파일 없음: {self.model_path} "
                  f"(학습 후 resource/ 에 배치 필요)")
            return
        try:
            from ultralytics import YOLO
            self.model = YOLO(self.model_path)
            self.names = self.model.names
            print(f"[YoloDetector] 로드 완료: {self.model_path}")
        except Exception as e:
            print(f"[YoloDetector] 로드 실패: {e}")

    @property
    def ready(self):
        return self.model is not None

    def detect(self, frame):
        """
        단일 프레임 추론.
        반환: [{'name','conf','box':[x1,y1,x2,y2],'cx','cy'}, ...]
        """
        if self.model is None or frame is None:
            return []
        results = self.model(frame, conf=self.conf, verbose=False)
        out = []
        for r in results:
            if r.boxes is None:
                continue
            for b in r.boxes:
                cls_id = int(b.cls[0])
                name = self.names.get(cls_id, str(cls_id)) \
                    if isinstance(self.names, dict) else self.model.names[cls_id]
                x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
                out.append({
                    "name": name,
                    "conf": float(b.conf[0]),
                    "box": [x1, y1, x2, y2],
                    "cx": (x1 + x2) / 2.0,
                    "cy": (y1 + y2) / 2.0,
                })
        return out

    def detect_stable(self, grab_frame_fn, num_frames: int = 5,
                       target: str = None):
        """
        다중프레임 탐지 후 가장 신뢰도 높은 누적 결과 반환.
        grab_frame_fn: 매 호출 시 새 프레임을 반환하는 콜러블.
        """
        agg = {}
        for _ in range(max(1, num_frames)):
            frame = grab_frame_fn()
            for d in self.detect(frame):
                if target and d["name"] != target:
                    continue
                key = d["name"]
                if key not in agg or d["conf"] > agg[key]["conf"]:
                    agg[key] = d
        return list(agg.values())
