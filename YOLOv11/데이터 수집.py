import cv2
import os
import time

def main():
    # 1. 이미지를 저장할 폴더 설정 (바탕화면에 yolo_dataset/images 폴더 생성)
    save_dir = "/home/yoo/yolo_dataset/images"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
        print(f"데이터 저장 폴더 생성 완료: {save_dir}")

    # 2. 로지텍 C920e 웹캠 연결
    cap = cv2.VideoCapture(1)
    
    # [핵심 추가] 하드웨어가 320x240 설정을 강제로 따르도록 MJPEG 포맷 지정
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    
    # 이제 카메라 기기가 이 해상도를 정확하게 인식합니다.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    # 버퍼가 쌓여서 옛날 프레임이 찍히는 현상을 방지하기 위해 버퍼 크기 최소화
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("에러: 웹캠을 열 수 없습니다. 카메라 권한 설정을 확인하세요.")
        return

    # 설정 변수
    target_count = 100       # 목표 저장 장수
    capture_interval = 0.5   # 사진 촬영 간격 (0.5초마다 1장씩 자동 저장)
    
    count = 0
    last_capture_time = time.time()

    print("\n=== 자동 데이터 수집 프로그램 시작 ===")
    print(f"- 목표 장수: {target_count}장")
    print(f"- 촬영 간격: {capture_interval}초마다 자동 저장")
    print("- 중간에 강제 종료하려면 영상 창에서 'q' 키를 누르세요.")
    print("========================================\n")

    while count < target_count:
        ret, frame = cap.read()
        if not ret:
            print("웹캠 프레임을 읽을 수 없습니다.")
            break

        current_time = time.time()
        
        # 설정한 간격(0.5초)이 지나면 자동으로 이미지 저장
        if current_time - last_capture_time >= capture_interval:
            timestamp = int(current_time * 1000)  # 파일명 중복 방지용 밀리초 타임스탬프
            filename = f"img_{timestamp}_{count + 1}.jpg"
            filepath = os.path.join(save_dir, filename)
            
            # 원본 프레임 저장
            cv2.imwrite(filepath, frame)
            count += 1
            print(f"[{count}/{target_count}] 자동 저장 완료: {filename}")
            
            # 마지막 촬영 시간 업데이트
            last_capture_time = current_time

        # 화면에 현재 저장 진행 상황 표시
        display_frame = frame.copy()
        cv2.putText(display_frame, f"Progress: {count} / {target_count}", (30, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        # 화면 보여주기 (창 크기가 320x240으로 작아져 딜레이가 사라집니다)
        cv2.imshow("Auto Data Collection - Logitech C920e", display_frame)

        # 'q' 키를 누르면 100장이 안 되었어도 중간에 종료 가능
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n사용자에 의해 중간에 종료되었습니다.")
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()
    
    if count >= target_count:
        print(f"\n성공: 목표치인 {target_count}장의 사진 수집이 완료되어 프로그램을 자동으로 종료합니다.")
    else:
        print(f"\n총 {count}장의 사진이 저장되었습니다.")

if __name__ == "__main__":
    main()
