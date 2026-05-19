import os
from glob import glob
from setuptools import find_packages, setup

package_name = "voice_processing"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        # [수정 완료] theme_map.yaml, .env 뿐만 아니라 .tflite 커스텀 모델 파일도 함께 share/resource로 빌드/설치되도록 지정
        (
            os.path.join("share", package_name, "resource"),
            glob("resource/*.yaml") + glob("resource/.env") + glob("resource/*.tflite"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="doocut_team",
    maintainer_email="team@doocut.local",
    description="인생DOO컷 음성 파이프라인: 웨이크워드 + Whisper STT + GPT 테마추출",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "get_keyword = voice_processing.get_keyword:main",
        ],
    },
)