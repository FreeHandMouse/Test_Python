# 遠くでも動く版

import cv2
import mediapipe as mp
import time
import webbrowser  # URLを開くために必要
import math        # 距離計算のために必要

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
url_opened_time = 0
url = "http://www.career.ce.uec.ac.jp/iccd/"  # 開きたいURLを指定

# 距離計算関数
def calculate_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

while True:
    if url_opened_time > 0 and time.time() - url_opened_time > 1:
        url_opened = False
        url_opened_time = 0
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

                # 親指の先端 (id: 4) と人差し指の先端 (id: 8)
                if id in [4, 8]:
                    cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)

            # 手の大きさ（親指付け根と小指付け根の距離）
            if 2 in landmarks and 17 in landmarks:
                hand_size = calculate_distance(landmarks[2][0], landmarks[2][1], landmarks[17][0], landmarks[17][1])

            # 親指と人差し指の先端の距離を手の大きさに対する割合で計算
            if 4 in landmarks and 8 in landmarks and 'hand_size' in locals():
                thumb_tip = landmarks[4]
                index_tip = landmarks[8]
                distance = calculate_distance(thumb_tip[0], thumb_tip[1], index_tip[0], index_tip[1])

                # 手の大きさに対する割合で比較（例：0.5以上でURLを開く）
                if distance / hand_size > 1.5 and not url_opened:
                    webbrowser.open(url)
                    url_opened = True  # URLを1回だけ開くようにする
                    url_opened_time = time.time()

            mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

    # FPS表示
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(img, f'FPS: {int(fps)}', (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    # 画像を表示
    cv2.imshow("Image", img)

    # 'q' キーで終了
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()