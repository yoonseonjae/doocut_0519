"""
qr_util.py
URL -> QR 코드 PNG 생성 유틸.
qrcode 라이브러리 사용. 미설치 시 명확한 에러 메시지 후 None 반환.
ROS2/DSR 비의존 순수 유틸.
"""

import os


def make_qr(data: str, out_path: str, box_size: int = 10,
            border: int = 4) -> str:
    """
    data(보통 URL)를 QR 로 만들어 out_path(png)에 저장.
    성공 시 out_path, 실패 시 None 반환.
    """
    if not data:
        return None
    try:
        import qrcode
    except Exception as e:
        print(f"[qr_util] qrcode 미설치: {e}")
        return None

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    qr = qrcode.QRCode(
        version=None,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=box_size,
        border=border,
    )
    qr.add_data(data)
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white")
    img.save(out_path)
    print(f"[qr_util] QR 저장: {out_path} ({data})")
    return out_path
