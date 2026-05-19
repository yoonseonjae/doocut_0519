# ThemeProps.msg
# 테마와 그 테마의 소품 리스트를 한 묶음으로 전달하는 메시지.
# voice_processing -> photo_booth_manager 토픽/필드 전달용.

string   theme            # 테마 키워드 (예: "beach", "princess")
string[] props            # 소품 클래스명 리스트
string   display_name     # 사람용 표시명 (예: "해변 휴양지")
string   tts_intro        # 테마 진입 안내 멘트 (TTS 노드용)
