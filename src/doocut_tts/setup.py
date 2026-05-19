from setuptools import find_packages, setup

package_name = "doocut_tts"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="doocut_team",
    maintainer_email="team@doocut.local",
    description="인생DOO컷 안내 방송 TTS 노드 (gTTS/pyttsx3 폴백)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tts_node = doocut_tts.tts_node:main",
        ],
    },
)
