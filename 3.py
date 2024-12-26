# こぶしを振るとURLを開くプログラム

import cv2
import mediapipe as mp
import time
import webbrowser  # URLを開くために必要
import numpy as np  # 配列操作のために必要

# カメラのキャプチャ
cap = cv2.VideoCapture(0)

# Mediapipe Handsモジュールのセットアップ
mpHands = mp.solutions.hands
hands = mpHands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
mpDraw = mp.solutions.drawing_utils

# 時間計測用
pTime = 0

# URLが開かれたかを確認するフラグ
url_opened = False
url = "https://www.uec.ac.jp"  # 開きたいURLを指定

# 手の位置を記録するためのリスト
positions = []
max_positions = 10  # 過去の位置を記録する数

# 振り動き検出の閾値
motion_threshold = 100

while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)  # ミラー表示
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
            landmarks = {}
            for id, lm in enumerate(handLms.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                landmarks[id] = (cx, cy)

            # 手首の座標 (ID: 0) を取得
            if 0 in landmarks:
                wrist_x, wrist_y = landmarks[0]

                # 手首の位置を記録
                positions.append(wrist_x)
                if len(positions) > max_positions:
                    positions.pop(0)

                # 過去の位置から動きを検出
                if len(positions) == max_positions:
                    # 手の左右の動きの幅を計算
                    motion_range = max(positions) - min(positions)

                    # 閾値を超えた場合、URLを開く
                    if motion_range > motion_threshold and not url_opened:
                        webbrowser.open(url)
                        url_opened = True  # URLを1回だけ開くようにする

            # ランドマークを描画
            mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

    # FPS表示
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(img, f'FPS: {int(fps)}', (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    # 画像を表示
    cv2.imshow("Image", img)

    # 'q' キーで終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
