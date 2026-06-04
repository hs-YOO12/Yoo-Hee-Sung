import cv2
import os
import time

def main():
    # 1. 윈도우 사용자 홈 디렉토리 하위에 폴더 생성 (C:\Users\유저명\yolo_dataset\images)
    user_home = os.path.expanduser("~")
    save_dir = os.path.join(user_home, "yolo_dataset", "images")
    
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
        print(f"데이터 저장 폴더 생성 완료: {save_dir}")

    # 2. 로지텍 C920e 웹캠 연결
    # 윈도우에서는 내장 캠과 외장 캠 순서가 다를 수 있습니다. (보통 0 또는 1)
    cap = cv2.VideoCapture(1) 
    
    # 로지텍 C920e의 하드웨어 해상도 제어를 위해 윈도우용 DSHOW 백엔드와 MJPEG 압축 강제 지시
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("에러: 웹캠을 열 수 없습니다. 카메라 인덱스(0을 1이나 2로)를 변경해보세요.")
        return

    target_count = 100       
    capture_interval = 0.5   
    count = 0
    last_capture_time = time.time()

    print("\n=== [Windows] 자동 데이터 수집 프로그램 시작 ===")
    print(f"- 저장 경로: {save_dir}")
    print(f"- 목표 장수: {target_count}장")
    print("========================================\n")

    while count < target_count:
        ret, frame = cap.read()
        if not ret:
            break

        current_time = time.time()
        if current_time - last_capture_time >= capture_interval:
            timestamp = int(current_time * 1000)
            filename = f"img_{timestamp}_{count + 1}.jpg"
            filepath = os.path.join(save_dir, filename)
            
            cv2.imwrite(filepath, frame)
            count += 1
            print(f"[{count}/{target_count}] 자동 저장 완료: {filename}")
            last_capture_time = current_time

        display_frame = frame.copy()
        cv2.putText(display_frame, f"Progress: {count} / {target_count}", (30, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow("Auto Data Collection - Windows", display_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
