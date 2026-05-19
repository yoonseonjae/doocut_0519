import os
import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = "voice_processing"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
SCENE_MAP_PATH = os.path.join(PACKAGE_PATH, "resource", "scene_map.yaml")


class SceneKeywordExtractor:
    """
    사용자 발화(STT 결과)에서 scene 키워드를 추출합니다.
    LLM 없이 scene_map.yaml의 keywords 목록으로 직접 매칭합니다.
    매칭 실패 시 LLM fallback을 사용합니다.
    """

    def __init__(self, llm=None):
        self.scene_map = self._load_scene_map()
        self.llm = llm  # 선택적 LLM fallback (get_keyword.py에서 주입)

    def _load_scene_map(self):
        with open(SCENE_MAP_PATH, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return data.get("scenes", {})

    def extract_scene(self, utterance: str) -> str | None:
        """
        발화에서 scene 이름을 반환합니다.
        1단계: scene_map.yaml keywords 직접 매칭
        2단계: LLM fallback (llm이 설정된 경우)
        반환: scene 이름 문자열 (예: "beach") 또는 None
        """
        utterance_lower = utterance.lower()

        # 1단계: 키워드 직접 매칭
        for scene_name, scene_data in self.scene_map.items():
            for keyword in scene_data.get("keywords", []):
                if keyword.lower() in utterance_lower:
                    print(f"[SceneExtractor] keyword match: '{keyword}' → scene: '{scene_name}'")
                    return scene_name

        # 2단계: LLM fallback
        if self.llm is not None:
            print("[SceneExtractor] No keyword match. Trying LLM fallback...")
            return self._llm_extract(utterance)

        print(f"[SceneExtractor] No scene matched for: '{utterance}'")
        return None

    def _llm_extract(self, utterance: str) -> str | None:
        """LLM을 통해 scene 이름을 추론합니다."""
        from langchain.prompts import PromptTemplate

        scene_list = list(self.scene_map.keys())
        prompt_template = PromptTemplate(
            input_variables=["scenes", "user_input"],
            template="""
당신은 사용자의 발화에서 장면/테마를 추출합니다.

<가능한 scene 목록>
{scenes}

<각 scene 설명>
- birthday: 생일, 파티, 생일 파티 관련 발화
- beach: 해변, 바닷가, 바다 관련 발화
- princess: 공주, 왕관, 마법 관련 발화

<규칙>
- 위 목록 중 하나만 반환하세요.
- 해당하는 scene이 없으면 "unknown"을 반환하세요.
- 설명 없이 scene 이름만 반환하세요.

<사용자 발화>
"{user_input}"
""",
        )
        chain = prompt_template | self.llm
        result = chain.invoke({"scenes": ", ".join(scene_list), "user_input": utterance})
        scene = result.content.strip().lower()
        if scene in self.scene_map:
            print(f"[SceneExtractor] LLM result: '{scene}'")
            return scene
        return None

    def get_props(self, scene_name: str) -> list[str]:
        """scene 이름에 해당하는 소품 목록을 반환합니다."""
        if scene_name in self.scene_map:
            return self.scene_map[scene_name].get("props", [])
        return []
