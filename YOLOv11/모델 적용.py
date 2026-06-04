import cv2
import os
from ultralytics import YOLO

def main():
    # 1. 학습 완료된 윈도우 내 'best.pt' 모델 경로 지정
    model_path = r"C:\Users\유희성\Desktop\best.pt"
    
    if not os.path.exists(model_path):
        print(f"에러: {model_path} 경로에서 모델 파일을 찾을 수 없습니다.")
        print("runs/detect/train/weights/ 폴더 내부에서 best.pt 파일을 해당 경로로 복사해주세요.")
        return

    print("=" * 50)
    print(" [Windows] 내가 만든 신호등 전용 YOLO11 모델을 로드합니다.")
    print("=" * 50)
    model = YOLO(model_path)

    # 2. 로지텍 C920e 웹캠 연결 (윈도우 전용 DSHOW 백엔드 명시)
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    
    # 윈도우 환경에서 C920e 기기 제어 및 딜레이 최소화를 위한 MJPEG 압축 포맷 강제 지정
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    
    # 해상도를 320x240으로 낮춰 완벽한 실시간 프레임 확보
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    # OpenCV 내부 낡은 프레임 버퍼 크기를 1개로 제한하여 싱크 밀림 방지
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("에러: 웹캠을 열 수 없습니다. 카메라 인덱스(0을 1이나 2로)를 확인하세요.")
        return

    print("\n웹캠 추론이 시작되었습니다! 영상 창을 클릭한 후 키보드 'q'를 누르면 종료됩니다.\n")

    while True:
        # [핵심 트릭] YOLO 분석 연산 도중 버퍼에 누적된 과거 프레임들을 순간적으로 싹 날립니다.
        # 이 처리를 해야 화면 잔상이 완벽히 사라지고 칼같이 실시간 화면이 유지됩니다.
        for _ in range(4):
            cap.grab()
            
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽어올 수 없습니다.")
            break

        # 3. 실시간 추론 수행 (stream=True 및 verbose=False 최적화 적용)
        # conf=0.4 옵션은 모델이 40% 이상 확신할 때만 화면에 박스를 칩니다.
        results = model(frame, stream=True, conf=0.4, verbose=False)

        # 4. 화면에 바운딩 박스와 클래스 라벨 그리기
        annotated_frame = frame
        for r in results:
            annotated_frame = r.plot()

        # 5. 최적화된 로지텍 웹캠 화면 표시 (320x240 크기로 딜레이 없이 송출)
        cv2.imshow("Windows - My Traffic Light YOLO11 Model", annotated_frame)

        # 'q' 키 누르면 안전하게 루프 탈출
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("실시간 테스트가 정상 종료되었습니다.")

if __name__ == "__main__":
    main()
