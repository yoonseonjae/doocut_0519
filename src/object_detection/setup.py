import os
from glob import glob
from setuptools import find_packages, setup

package_name = "object_detection"

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
        # best.pt / class_name.json / T_gripper2camera.npy 를 share/resource 로
        # 설치. 학습/캘리브 산출물을 resource/ 에 두면 그대로 패키징됨.
        (
            os.path.join("share", package_name, "resource"),
            [f for f in glob("resource/*")
             if os.path.isfile(f) and not f.endswith(package_name)],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="doocut_team",
    maintainer_email="team@doocut.local",
    description="인생DOO컷 비전: 소품 3D 탐지 + 얼굴기준 착용검사",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detection = object_detection.detection:main",
            "wearing_check = object_detection.wearing_check:main",
        ],
    },
)
