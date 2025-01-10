import ctypes
import math
import time

import cv2
import mediapipe as mp

user32 = ctypes.windll.user32

class MOUSEINPUT(ctypes.Structure):
    _fields_ = [("dx", ctypes.c_long),
                ("dy", ctypes.c_long),
                ("mouseData", ctypes.c_ulong),
                ("dwFlags", ctypes.c_ulong),
                ("time", ctypes.c_ulong),
                ("dwExtraInfo", ctypes.POINTER(ctypes.c_ulong))]
class INPUT(ctypes.Structure):
    _fields_ = [("type", ctypes.c_ulong), ("mi", MOUSEINPUT)]
     
LPINPUT = ctypes.POINTER(INPUT)
SendInput = user32.SendInput
SendInput.argtypes = (ctypes.c_uint, LPINPUT, ctypes.c_int)
SendInput.restype = ctypes.c_uint

# カメラのキャプチャ
cap = cv2.VideoCapture(1)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

screen_height = user32.GetSystemMetrics(1)
window_height = screen_height // 3
success, img = cap.read()
h, w, _ = img.shape
window_width = int(window_height * (w / h))

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
moving = False

# 距離計算関数
def calculate_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)
    temp_moving = False # 複数手のうち、少なくとも1つが動いているかどうか

    if results.multi_hand_landmarks:

        for handLms in results.multi_hand_landmarks: #1
            landmarks = {}
            # for id, lm in enumerate(handLms.landmark):
            for id in [0, 2, 4, 8, 12, 16, 17, 20]:
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

            if distance_0to2 / hand_size < distance_threshold and distance_0to3 / hand_size < distance_threshold and not temp_moving:

                # current_mouse_x, current_mouse_y = pyautogui.position()
                # current_mouse_x, current_mouse_y = win32api.GetCursorPos()
                wrist_x, wrist_y = landmarks[0] # 手首の位置
                positions.append([wrist_x, wrist_y])
                if len(positions) > max_positions:
                    positions.pop(0)

                # print (len(positions))
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

                    mouse_x = int(avg_motion_distance_x * kando)
                    mouse_y = int(avg_motion_distance_y * kando)
                    
                    _mi = MOUSEINPUT(mouse_x, mouse_y, 0, 0x0001, 0, None)
                    SendInput(1, INPUT(0,_mi), ctypes.sizeof(INPUT))

                moving = True
                temp_moving = True
            # else:
                # moving = False
                # positions = []

            mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)
        
        if not temp_moving:
            moving = False
            positions = []
        #1  
    
    else:
        # 手を認識していないとき
        time.sleep(0.000001)

    # FPS表示
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(img, f'FPS: {int(fps)}', (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
    cv2.putText(img, f'Moving: {moving}', (10, 140), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    # 画像を表示
    cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Image", window_width, window_height)
    cv2.imshow("Image", img)

    # 'q' キーで終了
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()