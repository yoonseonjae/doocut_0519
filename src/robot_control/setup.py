import os
from glob import glob
from setuptools import find_packages, setup

package_name = "robot_control"

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
        (
            os.path.join("share", package_name, "resource"),
            glob("resource/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="doocut_team",
    maintainer_email="team@doocut.local",
    description="인생DOO컷 로봇 제어: 소품 픽앤플 + 사람중심 동적 촬영 액션 서버",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_control = robot_control.robot_control:main",
            "photo_motion = robot_control.photo_motion:main",
        ],
    },
)
