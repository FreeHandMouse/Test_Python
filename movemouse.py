# 遠くでも動く版

import cv2
import mediapipe as mp
import time
import webbrowser  # URLを開くために必要
import math
import win32api

import pyautogui
pyautogui.FAILSAFE = False

# カメラのキャプチャ
cap = cv2.VideoCapture(1)

# Mediapipe Handsモジュールのセットアップ
mpHands = mp.solutions.hands
hands = mpHands.Hands()
mpDraw = mp.solutions.drawing_utils

# 時間計測用
pTime = 0
cTime = 0

# 手の位置を記録するためのリスト
positions = []
max_positions = 10  # 過去の位置を記録する数# 振り動き検出の閾値
motion_threshold = 150

# URLが開かれたかを確認するフラグ
moving = False
url_opened_time = 0
url1 = "http://www.career.ce.uec.ac.jp/iccd/"  # 開きたいURL1を指定
url2 = "https://www.uec.ac.jp"  # 開きたいURL2を指定

# 距離計算関数
def calculate_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

def mouse_move(x, y):
    win32api.SetCursorPos((x, y))

while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
            landmarks = {}
            # for id, lm in enumerate(handLms.landmark):
            for id in [0, 2, 4, 8, 12, 16, 17, 20]: # 手首、親指付け根、親指先端、人差し指先端、中指先端、薬指先端、小指付け根、小指先端
                lm = handLms.landmark[id]
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                landmarks[id] = (cx, cy)

                # 親指の先端 (id: 4) と人差し指の先端 (id: 8)
                if id in [4, 12, 16]:
                    cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)


            # 手の大きさ（親指付け根と小指付け根の距離）
            hand_size = calculate_distance(landmarks[2][0], landmarks[2][1], landmarks[17][0], landmarks[17][1])

            # 親指の先端と人差し指の先端の距離
            thumb_tip = landmarks[4]
            y2_tip = landmarks[12]
            y3_tip = landmarks[16]
            distance_0to2 = calculate_distance(thumb_tip[0], thumb_tip[1], y2_tip[0], y2_tip[1])
            distance_0to3 = calculate_distance(thumb_tip[0], thumb_tip[1], y3_tip[0], y3_tip[1])

            distance_threshold = 0.4

            if distance_0to2 / hand_size < distance_threshold and distance_0to3 / hand_size < distance_threshold:

                # current_mouse_x, current_mouse_y = pyautogui.position()
                current_mouse_x, current_mouse_y = win32api.GetCursorPos()
                wrist_x, wrist_y = landmarks[0] # 手首の位置
                positions.append([wrist_x, wrist_y])
                if len(positions) > max_positions:
                    positions.pop(0)

                if len(positions) > 1 and moving:
                    previous_wrist_x, previous_wrist_y = positions[-2]
                    motion_distance_x = wrist_x - previous_wrist_x
                    motion_distance_y = wrist_y - previous_wrist_y
                    
                    kando = 2

                    if len(positions) >= 4:
                        avg_motion_distance_x = sum([positions[i][0] - positions[i-1][0] for i in range(-1, -4, -1)]) / 3
                        avg_motion_distance_y = sum([positions[i][1] - positions[i-1][1] for i in range(-1, -4, -1)]) / 3
                    else:
                        avg_motion_distance_x = motion_distance_x
                        avg_motion_distance_y = motion_distance_y
                    
                    # print(avg_motion_distance_x, motion_distance_x)

                    mouse_x = current_mouse_x + avg_motion_distance_x * kando
                    mouse_y = current_mouse_y + avg_motion_distance_y * kando

                    mouse_x = int(mouse_x)
                    mouse_y = int(mouse_y)

                    # print(motion_distance_x, motion_distance_y)

                    # mouse_move_x = current_mouse_x + motion_distance_x*kando
                    # mouse_move_y = current_mouse_y + motion_distance_y*kando

                    # print(mouse_move_x, mouse_move_y)

                    # pyautogui.moveTo(mouse_move_x, mouse_move_y)
                    win32api.SetCursorPos((mouse_x, mouse_y))

                moving = True
            else:
                moving = False
                positions = []

            mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

    # FPS表示
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(img, f'FPS: {int(fps)}', (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
    cv2.putText(img, f'Moving: {moving}', (10, 140), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    # 画像を表示
    cv2.imshow("Image", img)

    # 'q' キーで終了
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()