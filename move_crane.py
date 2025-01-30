import ctypes
import time

import cv2
import mediapipe as mp
import serial

import calculate

sel = serial.Serial('COM3', 9600)

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
window_height = int(screen_height // 2)
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

pre_state = [[0,0,0,1],[0,0,0,1]]
p_time = time.time()

while True:
    time.sleep(0.1)
    plane_state = []
    success, img = cap.read()
    img = cv2.flip(img, 1)
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)
    temp_moving = False # 複数手のうち、少なくとも1つが動いているかどうか

    if results.multi_hand_landmarks:
        # with open('a.txt', 'w') as f:
        #     f.write(str(results))
        # filename = f'hand_landmarks.json'
        # with open(filename, 'w') as f:
        #     json.dump(results.multi_hand_landmarks, f, ensure_ascii=False, indent=2)

        for index, handLms in enumerate(results.multi_hand_landmarks): #1
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
            hand_size = calculate.distance(landmarks[2][0], landmarks[2][1], landmarks[17][0], landmarks[17][1])

            # 指の開閉状態を判定
            open_ste_th_1 = 30
            open_ste_th_2 = 80
            open_state = 0 # 0: 解放, 1: 動作 2: 閉じる
            y0_angle = calculate.yubi_angle_3d(landmarks,17,5,4)  # 親指
            y1_angle = calculate.yubi_angle_3d(landmarks,0,5,8)  # 人差し指
            y2_angle = calculate.yubi_angle_3d(landmarks,0,9,12)  # 中指
            y3_angle = calculate.yubi_angle_3d(landmarks,0,13,16)  # 薬指
            y4_angle = calculate.yubi_angle_3d(landmarks,0,17,20)  # 小指

            # print(int(y0_angle), int(y1_angle), int(y2_angle), int(y3_angle), int(y4_angle))

            if y1_angle > open_ste_th_2 and y2_angle > open_ste_th_2 and y3_angle > open_ste_th_2 and y4_angle > open_ste_th_2:
                open_state = 2
            elif y1_angle > open_ste_th_1 and y2_angle > open_ste_th_1 and y3_angle > open_ste_th_1 and y4_angle > open_ste_th_1:
                open_state = 1

            # print(open_state)

            if open_state >= 1 and not temp_moving:
              
                wrist_x, wrist_y, wrist_z = landmarks[0] # 手首の位置
                wrist_z = results.multi_hand_world_landmarks[index].landmark[0].z # zをworldで上書き
                # wrist_x, wrist_y, wrist_z = landmarks[4] # 親指の先端の位置
                positions.append([wrist_x, wrist_y, wrist_z])
                if len(positions) > max_positions:
                    positions.pop(0)

                # print (len(positions))
                if len(positions) > 1 and moving:
                    previous_wrist_x, previous_wrist_y, previous_wrist_z = positions[-2]
                    motion_distance_x = (wrist_x - previous_wrist_x) / hand_size
                    motion_distance_y = (wrist_y - previous_wrist_y) / hand_size
                    motion_distance_z = (wrist_z - previous_wrist_z) / hand_size
                    
                    move_th_1 = 0.1
                    move_th_2 = 0.08
                    move_th_world = 0.0001
                    
                    motion_distance_x = motion_distance_x if abs(motion_distance_x) > move_th_1 else 0
                    motion_distance_y = motion_distance_y if abs(motion_distance_y) > move_th_2 else 0
                    motion_distance_z = motion_distance_z if abs(motion_distance_z) > move_th_world else 0

                    move_x = calculate.sgn(motion_distance_x)
                    move_y = calculate.sgn(motion_distance_y) *-1
                    move_z = calculate.sgn(motion_distance_z)
                    arm = 1

                    if open_state == 2:
                        arm = 2
                    move_z = 0
                    plane_state = [move_x, move_z, move_y, arm]
                    # print(f'{move_x},{move_z},{move_y},{arm}\n')

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

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    output = [0,0,0,1]
    # if plane_state != 0 and pre_state != plane_state:
    #     p_time = cTime
    #     pre_state = plane_state
    # elif plane_state == 0 and cTime - p_time < 0.5:
    #     output = pre_state
    if plane_state == []:
        plane_state = [0,0,0,1]
    

    output[0] = plane_state[0] == pre_state[-1][0] and plane_state[0] or pre_state[-1][0]
    output[1] = plane_state[1] == pre_state[-1][1] and plane_state[1] or pre_state[-1][1]
    output[2] = plane_state[2] == pre_state[-1][2] and plane_state[2] or pre_state[-1][2]
    output[3] = plane_state[3] == pre_state[-1][3] and plane_state[3] or pre_state[-1][3]
    # output[3] = plane_state[3] if plane_state[3] == pre_state[0][3] else pre_state[0][3]
    
    pre_state.append(plane_state)
    pre_state.pop(0)

    sel.write(f'{output[0]},{output[1]},{output[2]},{output[3]}\n'.encode())
    print(pre_state, plane_state, output)

    # FPS表示
    # cTime = time.time()
    # fps = 1 / (cTime - pTime)
    # pTime = cTime
    # cv2.putText(img, f'FPS: {int(fps)}', (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
    cv2.putText(img, f'Moving: {moving}', (10, 60), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 3)

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