import os
from ultralytics import YOLO

def main():
    # 윈도우 스타일에 맞춘 절대 경로 지정
    dataset_dir = r"C:\Users\유희성\Desktop\asdf.yolov11"
    yaml_path = os.path.join(dataset_dir, "data.yaml")
    
    if not os.path.exists(yaml_path):
        print(f"에러: {yaml_path} 경로에 data.yaml 파일이 없습니다.")
        return

    print("==================================================")
    print(" YOLO11 신규 모델 학습을 시작합니다. (Windows)")
    print(f" 데이터셋 경로: {dataset_dir}")
    print("==================================================")

    # YOLO11 초경량 모델 로드
    model = YOLO("yolo11n.pt")

    # 학습 시작 (윈도우에서는 workers 속도를 위해 workers=0 또는 2를 추천합니다)
    model.train(
        data=yaml_path,
        epochs=50,
        imgsz=320,
        workers=2,
        device='cpu'  # 만약 엔비디아 외장 그래픽(RTX 등)이 없다면 'cpu'로 변경해주세요.
    )

if __name__ == "__main__":
    main()
