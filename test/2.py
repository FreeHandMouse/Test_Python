# ジェスチャー登録

import cv2
import mediapipe as mp
import time
import webbrowser
import math
import numpy as np
import json  # ファイル保存・読み込み用
import sys

sys.stdout.reconfigure(encoding='utf-8')
sys.stdin.reconfigure(encoding='utf-8')

# カメラのキャプチャ
cap = cv2.VideoCapture(1)

# Mediapipe Handsモジュールのセットアップ
mpHands = mp.solutions.hands
hands = mpHands.Hands()
mpDraw = mp.solutions.drawing_utils

# 時間計測用
pTime = 0
cTime = 0

# URLが開かれたかを確認するフラグ
url_opened = False
url = "https://www.uec.ac.jp"

# ジェスチャー登録用変数
registered_gesture = None
register_mode = False
gesture_file = "gesture.json"  # ジェスチャー保存ファイル名

# 距離計算関数
def calculate_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

# ジェスチャーの特徴を取得する関数
def get_landmark_distances(landmarks):
    base_landmark = landmarks[0]  # 手首の座標を基準にする
    distances = []
    for i in range(1, 21):  # 親指から小指までの20個のランドマーク
        distances.append(calculate_distance(base_landmark[0], base_landmark[1], landmarks[i][0], landmarks[i][1]))
    return np.array(distances)

# ジェスチャーをファイルに保存する関数
def save_gesture(gesture, filename):
    with open(filename, 'w') as f:
        json.dump(gesture.tolist(), f)
    print("ジェスチャーがファイルに保存されました！")

# ジェスチャーをファイルから読み込む関数
def load_gesture(filename):
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
        print("ジェスチャーがファイルから読み込まれました！")
        return np.array(data)
    except FileNotFoundError:
        print("保存されたジェスチャーが見つかりませんでした。")
        return None

# 起動時に保存されたジェスチャーを読み込む
registered_gesture = load_gesture(gesture_file)

while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
            landmarks = {}
            for id, lm in enumerate(handLms.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                landmarks[id] = (cx, cy)

                # 親指の先端 (id: 4)
                if id == 4:
                    cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
                # 小指の先端 (id: 20)
                if id == 20:
                    cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)

            # ジェスチャー登録モードの場合、ジェスチャーを保存
            if register_mode:
                registered_gesture = get_landmark_distances(landmarks)
                save_gesture(registered_gesture, gesture_file)
                register_mode = False

            # 登録されたジェスチャーと比較
            if registered_gesture is not None:
                current_gesture = get_landmark_distances(landmarks)
                difference = np.linalg.norm(current_gesture - registered_gesture)

                # ジェスチャーが一致する閾値を設定
                if difference < 30 and not url_opened:
                    webbrowser.open(url)
                    time.sleep(3)
                    # url_opened = True

            # Mediapipeでランドマークを描画
            mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

    # FPS表示
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(img, f'FPS: {int(fps)}', (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    # 画像を表示
    cv2.imshow("Image", img)

    # キー操作
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('r'):
        register_mode = True  # 'r'キーでジェスチャー登録モードに入る

cap.release()
cv2.destroyAllWindows()