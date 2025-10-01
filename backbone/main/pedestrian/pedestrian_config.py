#!/usr/bin/env python3

"""
보행자 모델 설정 파일
원하는 보행자 모델을 선택하여 사용할 수 있습니다.
"""

# 사용 가능한 보행자 모델들 (CARLA 0.9.15 기준)
PEDESTRIAN_MODELS = {
    # 기본 모델들
    "default_male": "walker.pedestrian.0001",
    "default_female": "walker.pedestrian.0002", 
    "casual_male": "walker.pedestrian.0003",
    "casual_female": "walker.pedestrian.0004",
    "business_male": "walker.pedestrian.0005",
    "business_female": "walker.pedestrian.0006",
    
    # 추가 모델들 (사용 가능한 경우)
    "young_male": "walker.pedestrian.0007",
    "young_female": "walker.pedestrian.0008",
    "elderly_male": "walker.pedestrian.0009",
    "elderly_female": "walker.pedestrian.0010",
    "athletic_male": "walker.pedestrian.0011",
    "athletic_female": "walker.pedestrian.0012",
    "casual_teen": "walker.pedestrian.0013",
    "student_female": "walker.pedestrian.0014",
    "professional_male": "walker.pedestrian.0015",
    "professional_female": "walker.pedestrian.0016",
    
    # 랜덤 선택
    "random": None
}

# 기본 설정
# DEFAULT_PEDESTRIAN = "random"  # 랜덤 선택
# DEFAULT_PEDESTRIAN = "default_female"  # 여성 보행자
DEFAULT_PEDESTRIAN = "casual_male"     # 캐주얼 남성 보행자
# DEFAULT_PEDESTRIAN = "business_female" # 비즈니스 여성 보행자

def get_pedestrian_model(selection=None):
    """
    보행자 모델 선택
    
    Args:
        selection (str): 선택할 보행자 모델 키
        
    Returns:
        str: 선택된 보행자 모델 ID 또는 None (랜덤)
    """
    if selection is None:
        selection = DEFAULT_PEDESTRIAN
    
    if selection == "random":
        return None
    
    if selection in PEDESTRIAN_MODELS:
        return PEDESTRIAN_MODELS[selection]
    else:
        print(f"⚠️ Unknown pedestrian model: {selection}")
        print(f"Available models: {list(PEDESTRIAN_MODELS.keys())}")
        return None

def list_available_models():
    """사용 가능한 보행자 모델 목록 출력"""
    print("👥 Available Pedestrian Models:")
    print("=" * 50)
    for key, value in PEDESTRIAN_MODELS.items():
        if value is None:
            print(f"  {key}: Random selection")
        else:
            print(f"  {key}: {value}")
    print("=" * 50)

if __name__ == "__main__":
    list_available_models()


