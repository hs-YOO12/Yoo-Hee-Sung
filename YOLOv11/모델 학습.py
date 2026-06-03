import os
from ultralytics import YOLO

def main():
    # 1. 데이터셋 경로 및 data.yaml 경로 설정
    dataset_dir = "/Users/yooheesung/Desktop/trafficlight.yolov11"
    yaml_path = os.path.join(dataset_dir, "data.yaml")
    
    # data.yaml 파일이 실제로 존재하는지 확인
    if not os.path.exists(yaml_path):
        print(f"에러: {yaml_path} 경로에 data.yaml 파일이 없습니다.")
        print("라벨링 툴에서 YOLO 포맷으로 정상적으로 다운로드 받았는지 확인해주세요.")
        return

    print("=" * 50)
    print(" YOLO11 신규 모델 학습을 시작합니다.")
    print(f" 데이터셋 경로: {dataset_dir}")
    print("=" * 50)

    # 2. 베이스가 될 YOLO11 나노(nano) 모델 로드 (가장 가볍고 속도가 빠름)
    # 처음 실행 시 인터넷에서 'yolo11n.pt' 가중치 파일을 자동으로 다운로드합니다.
    model = YOLO("yolo11n.pt")

    # 3. 모델 학습 시작
    # 주요 파라미터 설명:
    # - data: 데이터셋의 정보가 담긴 yaml 파일 경로
    # - epochs: 전체 데이터셋을 몇 번 반복해서 학습할지 설정 (데이터가 100장이면 30~50회 추천)
    # - imgsz: 입력 이미지 해상도 (기본값 640)
    # - batch: 한 번에 학습할 데이터 묶음 크기 (맥 환경의 메모리에 맞춰 16 또는 8 추천, -1은 자동 설정)
    # - device: 맥북 칩셋(M1/M2/M3 등 Apple Silicon)의 GPU 가속을 쓰기 위해 'mps' 지정 (인텔 맥인 경우 'cpu'로 변경)
    model.train(
        data=yaml_path,
        epochs=50,
        imgsz=640,
        batch=16,
        device="mps"  # Apple Silicon GPU 가속 적용
    )

    print("\n" + "=" * 50)
    print(" 학습이 완료되었습니다!")
    print(" 학습 결과 및 최종 모델(.pt)은 현재 폴더 안의 'runs/detect/train/weights/'에 저장됩니다.")
    print(" 그 중 'best.pt' 파일을 사용하시면 됩니다.")
    print("=" * 50)

if __name__ == "__main__":
    main()
