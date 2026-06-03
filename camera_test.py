import cv2
import os
from ultralytics import YOLO

def main():
    # 1. 방금 학습 완료된 'best.pt' 모델 경로 지정
    # 만약 학습을 여러 번 반복했다면 train2, train3 등으로 숫자가 붙을 수 있으니 폴더명을 확인해주세요.
    model_path = "/Users/yooheesung/Desktop/best.pt"
    
    if not os.path.exists(model_path):
        print(f"에러: {model_path} 경로에서 모델 파일을 찾을 수 없습니다.")
        print("runs/detect/ 폴더 내부를 확인하여 정확한 train 폴더명을 적어주세요.")
        return

    print("=" * 50)
    print(" 내가 만든 신호등 전용 YOLO11 모델을 로드합니다.")
    print("=" * 50)
    model = YOLO(model_path)

    # 2. 로지텍 C920e 웹캠 연결 (안 나오면 0을 1이나 2로 변경)
    cap = cv2.VideoCapture(0)
    
    # 해상도 설정 (1280x720 HD)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("에러: 웹캠을 열 수 없습니다. 카메라 연결 및 보안 설정을 확인하세요.")
        return

    print("\n웹캠이 켜졌습니다! 영상 창을 클릭한 후 키보드 'q'를 누르면 종료됩니다.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽어올 수 없습니다.")
            break

        # 3. 실시간 추론 수행 (conf 옵션으로 인식 민감도 조절 가능, 예: conf=0.5는 50% 이상 확실할 때만 표시)
        results = model(frame, stream=True, conf=0.4)

        # 4. 화면에 바운딩 박스와 라벨 그리기
        annotated_frame = frame
        for r in results:
            annotated_frame = r.plot()

        # 5. 로지텍 웹캠 화면 표시
        cv2.imshow("Logitech C920e - My Traffic Light Model", annotated_frame)

        # 'q' 키 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("테스트가 정상 종료되었습니다.")

if __name__ == "__main__":
    main()