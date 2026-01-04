# 라이브러리 설치
pip install tensorflow-macos
pip install mediapipe

# 데이터 로드 및 전처리
import os
from tensorflow.keras.preprocessing.image import load_img, img_to_array
import numpy as np
from tensorflow.keras.utils import to_categorical
from sklearn.model_selection import train_test_split

dataset_dir = 'Drowsy_datset'

train_dir = os.path.join(dataset_dir, 'train')
test_dir = os.path.join(dataset_dir, 'test')

train_drowsy_dir = os.path.join(train_dir, 'DROWSY')
train_natural_dir = os.path.join(train_dir, 'NATURAL')
test_drowsy_dir = os.path.join(test_dir, 'DROWSY')
test_natural_dir = os.path.join(test_dir, 'NATURAL')

img_size = (224, 224)

def load_images_from_folder(folder, label):
    images = []
    labels = []
    for filename in os.listdir(folder):
        img_path = os.path.join(folder, filename)
        img = load_img(img_path, target_size=img_size)
        img_array = img_to_array(img)
        images.append(img_array)
        labels.append(label)
    return images, labels

train_images_drowsy, train_labels_drowsy = load_images_from_folder(train_drowsy_dir, 1)
train_images_natural, train_labels_natural = load_images_from_folder(train_natural_dir, 0)
test_images_drowsy, test_labels_drowsy = load_images_from_folder(test_drowsy_dir, 1)
test_images_natural, test_labels_natural = load_images_from_folder(test_natural_dir, 0)

train_images = np.array(train_images_drowsy + train_images_natural)
train_labels = np.array(train_labels_drowsy + train_labels_natural)
test_images = np.array(test_images_drowsy + test_images_natural)
test_labels = np.array(test_labels_drowsy + test_labels_natural)

train_labels = to_categorical(train_labels, 2)
test_labels = to_categorical(test_labels, 2)

train_images = train_images / 255.0
test_images = test_images / 255.0

X_train, X_val, y_train, y_val = train_test_split(train_images, train_labels, test_size=0.2, random_state=42)

print(f"훈련 데이터 크기: {X_train.shape}, 검증 데이터 크기: {X_val.shape}, 테스트 데이터 크기: {test_images.shape}")

# CNN 설계
from tensorflow.keras import layers, models
from tensorflow.keras.models import Model
from tensorflow.keras.applications import VGG16

model = models.Sequential([
    layers.Conv2D(32, (3, 3), activation='relu', input_shape=(224, 224, 3)),
    layers.MaxPooling2D((2, 2)),
    layers.Conv2D(64, (3, 3), activation='relu'),
    layers.MaxPooling2D((2, 2)),
    layers.Conv2D(128, (3, 3), activation='relu'),
    layers.MaxPooling2D((2, 2)),
    layers.Conv2D(256, (3, 3), activation='relu'),
    layers.MaxPooling2D((2, 2)),
    layers.Conv2D(512, (3, 3), activation='relu'),
    layers.MaxPooling2D((2, 2)),

    # GlobalAveragePooling2D 사용
    layers.GlobalAveragePooling2D(),
    layers.Dense(256, activation='relu'),
    layers.Dropout(0.1),
    layers.Dense(2, activation='softmax')  # DROWSY와 NATURAL 두 클래스  
])

model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

model.summary()

# 모델 학습
from tensorflow.keras.callbacks import EarlyStopping

early_stopping = EarlyStopping(monitor='val_loss', patience=3, restore_best_weights=True)

history = model.fit(X_train, y_train, epochs=10, batch_size=32, validation_data=(X_val, y_val))

# 훈련 결과 시각화
import matplotlib.pyplot as plt

plt.plot(history.history['accuracy'], label='Training Accuracy')
plt.plot(history.history['val_accuracy'], label='Validation Accuracy')
plt.xlabel('Epochs')
plt.ylabel('Accuracy')
plt.legend()
plt.show()

plt.plot(history.history['loss'], label='Training Loss')
plt.plot(history.history['val_loss'], label='Validation Loss')
plt.xlabel('Epochs')
plt.ylabel('Loss')
plt.legend()
plt.show()

# 모델 저장
model.save('drowsy_detection_model.h5')
print(os.path.abspath('drowsy_detection_model.h5'))

# 실시간 졸음 감지 및 경고 시스템
import cv2
import mediapipe as mp
from tensorflow.keras.models import load_model

model = load_model('/Users/yooheesung/Desktop/drowsy_detection_model.h5')

cap = cv2.VideoCapture(0)

mp_face_detection = mp.solutions.face_detection
mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils

face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.2)
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.2, min_tracking_confidence=0.2)

def preprocess_frame(frame):
    image = cv2.resize(frame, (224, 224))
    image = np.array(image) / 255.0
    image = np.expand_dims(image, axis=0)
    return image

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    results = face_detection.process(rgb_frame)
    mesh_results = face_mesh.process(rgb_frame)

    if results.detections:
        for detection in results.detections:
            # 얼굴 주위에 사각형 그리기
            mp_drawing.draw_detection(frame, detection)

        if mesh_results.multi_face_landmarks:
            for face_landmarks in mesh_results.multi_face_landmarks:

                face_image = frame 
                preprocessed_face = preprocess_frame(face_image)

                prediction = model.predict(preprocessed_face)
                print("Prediction probabilities:", prediction)

                if prediction[0][0] > 0.3:
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    text = "Warning! Wake up!"
                    position = (50, 50)
                    font_scale = 1
                    color = (0, 0, 255)
                    thickness = 2
                    cv2.putText(frame, text, position, font, font_scale, color, thickness)
                else:
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    text = "No problem!"
                    position = (50, 50)
                    font_scale = 1
                    color = (0, 255, 0)
                    thickness = 2
                    cv2.putText(frame, text, position, font, font_scale, color, thickness)

    cv2.imshow('Drowsy Driver Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
