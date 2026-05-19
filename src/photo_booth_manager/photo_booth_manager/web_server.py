"""
web_server.py
완성 콜라주를 노출하는 Flask 정적 웹 서버.

  - static/output/ 의 결과 이미지를 URL 로 서빙
  - /  : 최신 결과 표시 페이지 (templates/result.html)
  - /img/<name> : 결과 이미지 원본
  - 백그라운드 스레드로 기동 (manager 가 호출)
  - 선택적으로 ngrok 터널 (pyngrok 설치 시) -> 외부 접속 URL

ROS2/DSR 비의존. manager_node 가 import 해 사용.
"""

import os
import threading

_LATEST = {"image": None, "theme": None}


def set_latest(image_name: str, theme: str = ""):
    _LATEST["image"] = image_name
    _LATEST["theme"] = theme


def _build_app(output_dir, templates_dir):
    from flask import Flask, send_from_directory, render_template_string

    app = Flask(__name__)

    default_html = """
    <!doctype html><html lang="ko"><head><meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>인생DOO컷</title>
    <style>
      body{font-family:sans-serif;text-align:center;background:#fafafa;margin:0;padding:24px;}
      h1{color:#333;} img{max-width:92vw;border-radius:12px;
      box-shadow:0 4px 18px rgba(0,0,0,.15);}
      .empty{color:#999;margin-top:40px;}
    </style></head><body>
    <h1>인생DOO컷 📸</h1>
    {% if image %}
      <p>테마: {{theme}}</p>
      <img src="/img/{{image}}" alt="result">
    {% else %}
      <p class="empty">아직 완성된 사진이 없습니다.</p>
    {% endif %}
    </body></html>
    """

    @app.route("/")
    def index():
        tmpl_file = os.path.join(templates_dir or "", "result.html")
        if templates_dir and os.path.exists(tmpl_file):
            with open(tmpl_file, "r", encoding="utf-8") as f:
                html = f.read()
        else:
            html = default_html
        return render_template_string(
            html, image=_LATEST["image"], theme=_LATEST["theme"])

    @app.route("/img/<path:name>")
    def img(name):
        return send_from_directory(output_dir, name)

    return app


class WebServer:
    def __init__(self, output_dir, templates_dir=None,
                 host="0.0.0.0", port=8080, use_ngrok=False):
        self.output_dir = os.path.abspath(output_dir)
        self.templates_dir = templates_dir
        self.host = host
        self.port = port
        self.use_ngrok = use_ngrok
        self.public_url = f"http://{host}:{port}"
        self._thread = None
        os.makedirs(self.output_dir, exist_ok=True)

    def _maybe_ngrok(self):
        if not self.use_ngrok:
            return
        try:
            from pyngrok import ngrok
            tunnel = ngrok.connect(self.port, "http")
            self.public_url = tunnel.public_url
            print(f"[web_server] ngrok URL: {self.public_url}")
        except Exception as e:
            print(f"[web_server] ngrok 비활성: {e}")

    def start(self):
        """백그라운드 스레드로 Flask 기동."""
        try:
            app = _build_app(self.output_dir, self.templates_dir)
        except Exception as e:
            print(f"[web_server] Flask 미설치/초기화 실패: {e}")
            return None

        self._maybe_ngrok()

        def _run():
            app.run(host=self.host, port=self.port,
                    debug=False, use_reloader=False)

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()
        print(f"[web_server] 시작: {self.public_url}")
        return self.public_url

    def publish_result(self, image_path, theme=""):
        """결과 이미지를 output_dir 로 복사 후 최신으로 등록, URL 반환."""
        import shutil
        name = os.path.basename(image_path)
        dst = os.path.join(self.output_dir, name)
        if os.path.abspath(image_path) != os.path.abspath(dst):
            shutil.copy(image_path, dst)
        set_latest(name, theme)
        return f"{self.public_url}/img/{name}"
