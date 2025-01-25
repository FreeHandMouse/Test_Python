import ctypes
import math
import time

import cv2
import mediapipe as mp
import serial

# sel = serial.Serial('COM3', 9600)

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
RClicked = False
LClicked = False

def calculate_angle_3d(vector1, vector2):
    # 内積を計算
    dot_product = (
        vector1[0] * vector2[0]
        + vector1[1] * vector2[1]
        + vector1[2] * vector2[2]
    )
    # ベクトルの大きさを計算
    magnitude1 = math.sqrt(vector1[0] ** 2 + vector1[1] ** 2 + vector1[2] ** 2)
    magnitude2 = math.sqrt(vector2[0] ** 2 + vector2[1] ** 2 + vector2[2] ** 2)
    # 角度を計算（ラジアン単位）
    if magnitude1 == 0 or magnitude2 == 0:
        return 180
    angle_rad = math.acos(dot_product / (magnitude1 * magnitude2))
    # ラジアンを度に変換
    angle_deg = math.degrees(angle_rad)
    return angle_deg

def yubi_angle_3d(landmarks, posnum1, posnum2, posnum3):
    pos1 = landmarks[posnum1]
    pos2 = landmarks[posnum2]
    pos3 = landmarks[posnum3]
    atob = (pos2[0] - pos1[0], pos2[1] - pos1[1], pos2[2] - pos1[2])
    btoc = (pos3[0] - pos2[0], pos3[1] - pos2[1], pos3[2] - pos2[2])
    angle = calculate_angle_3d(atob, btoc)
    return angle

# 距離計算関数
def calculate_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

def sgn(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0

while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)
    temp_moving = False # 複数手のうち、少なくとも1つが動いているかどうか

    if results.multi_hand_landmarks:
        with open('a.txt', 'w') as f:
            f.write(str(results))
        # filename = f'hand_landmarks.json'
        # with open(filename, 'w') as f:
        #     json.dump(results.multi_hand_landmarks, f, ensure_ascii=False, indent=2)

        for handLms in results.multi_hand_landmarks: #1
            landmarks = {}
            # for id, lm in enumerate(handLms.landmark):
            for id in range(21):
                lm = handLms.landmark[id]
                h, w, c = img.shape
                cx, cy, cz = int(lm.x * w), int(lm.y * h), int(lm.z * w)
                landmarks[id] = (cx, cy, cz)

                # 親指の先端 (id: 4) と人差し指の先端 (id: 8)
                if id in [4, 12, 16, 8, 20]:
                    cv2.circle(img, (cx, cy), 10, (255, 0, 255), cv2.FILLED)


            # 手の大きさ（親指付け根と小指付け根の距離）
            hand_size = calculate_distance(landmarks[2][0], landmarks[2][1], landmarks[17][0], landmarks[17][1])

            # 指の開閉状態を判定
            open_ste_th_1 = 30
            open_state = 0 # 0: 解放, 1: 動作 2: 閉じる
            y0_angle = yubi_angle_3d(landmarks,17,5,4)  # 親指
            y1_angle = yubi_angle_3d(landmarks,0,5,8)  # 人差し指
            y2_angle = yubi_angle_3d(landmarks,0,9,12)  # 中指
            y3_angle = yubi_angle_3d(landmarks,0,13,16)  # 薬指
            y4_angle = yubi_angle_3d(landmarks,0,17,20)  # 小指

            print(int(y0_angle), int(y1_angle), int(y2_angle), int(y3_angle), int(y4_angle))

            if y1_angle > open_ste_th_1 and y2_angle > open_ste_th_1 and y3_angle > open_ste_th_1 and y4_angle > open_ste_th_1:
                open_state = 1

            # 親指の先端と人差し指の先端の距離
            y0_tip = landmarks[4]
            y1_tip = landmarks[8]
            y2_tip = landmarks[12]
            y3_tip = landmarks[16]
            y4_tip = landmarks[20]
            distance_0to2 = calculate_distance(y0_tip[0], y0_tip[1], y2_tip[0], y2_tip[1])
            distance_0to3 = calculate_distance(y0_tip[0], y0_tip[1], y3_tip[0], y3_tip[1])
            distance_1to2 = calculate_distance(y1_tip[0], y1_tip[1], y2_tip[0], y2_tip[1])
            distance_3to4 = calculate_distance(y3_tip[0], y3_tip[1], y4_tip[0], y4_tip[1])

            distance_threshold = 0.35

            if distance_0to2 / hand_size < distance_threshold and distance_0to3 / hand_size < distance_threshold:
                if open_state == 1:
                    open_state = 2
            print(open_state)

            if distance_0to2 / hand_size < distance_threshold and distance_0to3 / hand_size < distance_threshold and not temp_moving:
              
                # current_mouse_x, current_mouse_y = pyautogui.position()
                # current_mouse_x, current_mouse_y = win32api.GetCursorPos()
                # wrist_x, wrist_y = landmarks[0] # 手首の位置
                wrist_x, wrist_y, wrist_z = landmarks[4] # 親指の先端の位置
                positions.append([wrist_x, wrist_y])
                if len(positions) > max_positions:
                    positions.pop(0)

                # print (len(positions))
                if len(positions) > 1 and moving:
                    previous_wrist_x, previous_wrist_y = positions[-2]
                    motion_distance_x = wrist_x - previous_wrist_x
                    motion_distance_y = wrist_y - previous_wrist_y
                    
                    # kando = 150
                    kando = 150
                    avg_num = 3

                    if len(positions) >= avg_num+1:
                        avg_motion_distance_x = sum([positions[i][0] - positions[i-1][0] for i in range(-1, -(avg_num+1), -1)]) / avg_num / hand_size
                        avg_motion_distance_y = sum([positions[i][1] - positions[i-1][1] for i in range(-1, -(avg_num+1), -1)]) / avg_num / hand_size
                    else:
                        avg_motion_distance_x = motion_distance_x / hand_size
                        avg_motion_distance_y = motion_distance_y / hand_size
                    
                    acceleration_x = 1
                    acceleration_y = 1

                    mouse_x = int(avg_motion_distance_x * kando * acceleration_x)
                    mouse_y = int(avg_motion_distance_y * kando * acceleration_x)

                moving = True
                temp_moving = True

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