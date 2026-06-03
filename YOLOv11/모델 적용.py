import cv2
import os
from ultralytics import YOLO

def main():
    # 1. 학습 완료된 'best.pt' 모델 경로 지정
    model_path = "/home/yoo/best.pt"
    
    if not os.path.exists(model_path):
        print(f"에러: {model_path} 경로에서 모델 파일을 찾을 수 없습니다.")
        return

    print("=" * 50)
    print(" 내가 만든 신호등 전용 YOLO11 모델을 로드합니다.")
    print("=" * 50)
    model = YOLO(model_path)

    # 2. 로지텍 C920e 웹캠 연결
    cap = cv2.VideoCapture(1)
    
    # [핵심] 하드웨어 해상도 변경 허용 및 딜레이 감소를 위해 MJPEG 포맷 강제 지정
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    
    # 이제 320x240 해상도 설정이 카메라 기기에 칼같이 반영됩니다.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    # OpenCV 내부 낡은 프레임 버퍼 크기를 1개로 제한
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("에러: 웹캠을 열 수 없습니다. 카메라 연결 번호(1 또는 2)를 확인하세요.")
        return

    print("\n웹캠이 켜졌습니다! 영상 창을 클릭한 후 키보드 'q'를 누르면 종료됩니다.\n")

    while True:
        # [핵심 트릭] YOLO 추론이 도는 동안 버퍼에 쌓인 과거 프레임을 순간적으로 싹 날립니다.
        # 이 작업을 거쳐야 잔상이 완전히 사라지고 무조건 '현재 순간'만 찍힙니다.
        for _ in range(4):
            cap.grab()
            
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽어올 수 없습니다.")
            break

        # 3. 실시간 추론 수행 (stream=True 일 때는 리스트가 아니므로 아래와 같이 핸들링)
        results = model(frame, stream=True, conf=0.4, verbose=False)

        # 4. 화면에 바운딩 박스와 라벨 그리기
        annotated_frame = frame
        for r in results:
            annotated_frame = r.plot()

        # 5. 최적화된 로지텍 웹캠 화면 표시 (창 크기가 확실하게 320x240으로 줄어듭니다)
        cv2.imshow("Logitech C920e - My Traffic Light Model", annotated_frame)

        # 'q' 키 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("테스트가 정상 종료되었습니다.")

if __name__ == "__main__":
    main()
