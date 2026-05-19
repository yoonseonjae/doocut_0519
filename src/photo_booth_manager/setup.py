import os
from glob import glob
from setuptools import find_packages, setup

package_name = "photo_booth_manager"

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
            os.path.join("share", package_name, "templates"),
            glob("templates/*"),
        ),
        (
            os.path.join("share", package_name, "static", "frames"),
            [f for f in glob("static/frames/*") if os.path.isfile(f)],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="doocut_team",
    maintainer_email="team@doocut.local",
    description="인생DOO컷 마스터 파이프라인: 음성->비전->로봇->합성->QR + 웹서버",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "manager_node = photo_booth_manager.manager_node:main",
        ],
    },
)
