import cv2
import mediapipe as mp
import time
import webbrowser  # URLを開くために必要
import math        # 距離計算のために必要

# カメラのキャプチャ
cap = cv2.VideoCapture(0)

# Mediapipe Handsモジュールのセットアップ
mpHands = mp.solutions.hands
hands = mpHands.Hands()
mpDraw = mp.solutions.drawing_utils

# 時間計測用
pTime = 0
cTime = 0

# URLが開かれたかを確認するフラグ
url_opened = False
url = "https://trpfrog.net"  # 開きたいURLを指定

# 距離計算関数
def calculate_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

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

            # 親指と小指の先端の距離を計算
            if 4 in landmarks and 20 in landmarks:
                thumb_tip = landmarks[4]
                pinky_tip = landmarks[20]
                distance = calculate_distance(thumb_tip[0], thumb_tip[1], pinky_tip[0], pinky_tip[1])
                
                # 距離が200ピクセル以上ならURLを開く
                if distance > 200 and not url_opened:
                    webbrowser.open(url)
                    url_opened = True  # URLを1回だけ開くようにする

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