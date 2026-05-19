"""
collage.py
4컷 -> 인생네컷 세로 프레임 콜라주 합성.

  - PIL 우선 사용 (없으면 OpenCV 폴백)
  - 테마별 프레임 템플릿(static/frames/<theme>.png) 있으면 오버레이
  - 프레임 없으면 단색 배경 + 캡션으로 자동 생성

ROS2/DSR 비의존 순수 유틸.
"""

import os
import datetime


# 인생네컷 표준 비율(세로 2x2 또는 1x4). 여기선 2x2 세로형.
CANVAS_W = 1200
CANVAS_H = 1800
MARGIN = 40
GAP = 30

THEME_BG = {
    "beach": (135, 206, 235),       # 하늘색
    "princess": (255, 220, 240),    # 연분홍
    "default": (245, 245, 245),
}


def _pil_collage(image_paths, out_path, theme, frame_path):
    from PIL import Image, ImageDraw, ImageFont

    bg = THEME_BG.get(theme, THEME_BG["default"])
    canvas = Image.new("RGB", (CANVAS_W, CANVAS_H), bg)

    cell_w = (CANVAS_W - 2 * MARGIN - GAP) // 2
    cell_h = (CANVAS_H - 2 * MARGIN - GAP - 120) // 2  # 하단 캡션 영역 120

    positions = [
        (MARGIN, MARGIN),
        (MARGIN + cell_w + GAP, MARGIN),
        (MARGIN, MARGIN + cell_h + GAP),
        (MARGIN + cell_w + GAP, MARGIN + cell_h + GAP),
    ]

    for i, p in enumerate(image_paths[:4]):
        if not os.path.exists(p):
            continue
        im = Image.open(p).convert("RGB")
        im = _fit_crop(im, cell_w, cell_h)
        canvas.paste(im, positions[i])

    draw = ImageDraw.Draw(canvas)
    caption = f"인생DOO컷  |  {theme}  |  " \
              f"{datetime.datetime.now().strftime('%Y-%m-%d')}"
    try:
        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/nanum/NanumGothic.ttf", 44)
    except Exception:
        font = ImageFont.load_default()
    draw.text((MARGIN, CANVAS_H - 90), caption, fill=(60, 60, 60), font=font)

    # 테마 프레임 오버레이 (투명 PNG)
    if frame_path and os.path.exists(frame_path):
        try:
            frame = Image.open(frame_path).convert("RGBA")
            frame = frame.resize((CANVAS_W, CANVAS_H))
            canvas = canvas.convert("RGBA")
            canvas.alpha_composite(frame)
            canvas = canvas.convert("RGB")
        except Exception:
            pass

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    canvas.save(out_path, quality=95)
    return out_path


def _fit_crop(im, w, h):
    """비율 유지 center-crop 후 (w,h)로 리사이즈."""
    from PIL import Image
    src_w, src_h = im.size
    scale = max(w / src_w, h / src_h)
    nw, nh = int(src_w * scale), int(src_h * scale)
    im = im.resize((nw, nh), Image.LANCZOS)
    left = (nw - w) // 2
    top = (nh - h) // 2
    return im.crop((left, top, left + w, top + h))


def _cv_collage(image_paths, out_path, theme):
    """PIL 미설치 시 OpenCV 폴백 (캡션 한글은 생략)."""
    import cv2
    import numpy as np

    bg = THEME_BG.get(theme, THEME_BG["default"])[::-1]  # RGB->BGR
    canvas = np.full((CANVAS_H, CANVAS_W, 3), bg, dtype=np.uint8)

    cell_w = (CANVAS_W - 2 * MARGIN - GAP) // 2
    cell_h = (CANVAS_H - 2 * MARGIN - GAP - 120) // 2
    positions = [
        (MARGIN, MARGIN),
        (MARGIN + cell_w + GAP, MARGIN),
        (MARGIN, MARGIN + cell_h + GAP),
        (MARGIN + cell_w + GAP, MARGIN + cell_h + GAP),
    ]
    for i, p in enumerate(image_paths[:4]):
        if not os.path.exists(p):
            continue
        img = cv2.imread(p)
        if img is None:
            continue
        img = cv2.resize(img, (cell_w, cell_h))
        x, y = positions[i]
        canvas[y:y + cell_h, x:x + cell_w] = img

    cv2.putText(canvas, f"Life DOO-Cut | {theme}",
                (MARGIN, CANVAS_H - 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (60, 60, 60), 2)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    cv2.imwrite(out_path, canvas)
    return out_path


def make_collage(image_paths, out_path, theme="default",
                  frames_dir=None) -> str:
    """
    4컷 합성 진입점.
    image_paths: 원본 4장 경로 리스트
    out_path: 결과 저장 경로 (png/jpg)
    theme: 테마명 (배경/프레임 선택)
    frames_dir: static/frames 경로 (테마 프레임 png 탐색)
    반환: 성공 시 out_path, 실패 시 None
    """
    frame_path = None
    if frames_dir:
        cand = os.path.join(frames_dir, f"{theme}.png")
        if os.path.exists(cand):
            frame_path = cand

    try:
        return _pil_collage(image_paths, out_path, theme, frame_path)
    except Exception as e_pil:
        print(f"[collage] PIL 합성 실패, OpenCV 폴백: {e_pil}")
        try:
            return _cv_collage(image_paths, out_path, theme)
        except Exception as e_cv:
            print(f"[collage] OpenCV 합성도 실패: {e_cv}")
            return None
