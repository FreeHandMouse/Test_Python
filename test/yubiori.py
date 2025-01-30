import cv2
import mediapipe as mp
import math

# MediaPipeのセットアップ
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils


# ベクトル間の角度を計算する関数
def calculate_angle(vector1, vector2):
    # 内積を計算
    dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1]
    # ベクトルの大きさを計算
    magnitude1 = math.sqrt(vector1[0] ** 2 + vector1[1] ** 2)
    magnitude2 = math.sqrt(vector2[0] ** 2 + vector2[1] ** 2)
    # 角度を計算（ラジアン単位）
    if magnitude1 == 0 or magnitude2 == 0:
        return 180  # ベクトルの大きさがゼロの場合、角度は180°とみなす
    angle_rad = math.acos(dot_product / (magnitude1 * magnitude2))
    # ラジアンを度に変換
    angle_deg = math.degrees(angle_rad)
    return angle_deg

def calculate_vector(vectora, vectorb, vectorc):
    atob = (vectorb[0]-vectora[0], vectorb[1]-vectora[1])
    btoc = (vectorc[0]-vectorb[0], vectorc[1]-vectorb[1])
    angle = calculate_angle(atob,btoc)
    is_angle_below_90 = angle <= 100
    return is_angle_below_90

# 距離計算関数ああ
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def oyayubi(vectora, vectorb, vectorc):
    atob = (vectorb[0]-vectora[0], vectorb[1]-vectora[1])
    btoc = (vectorc[0]-vectorb[0], vectorc[1]-vectorb[1])
    angle = calculate_angle(atob,btoc)
    is_angle_below_90 = angle <= 30
    print(angle)
    return is_angle_below_90

# 開閉状態判定関数
def is_finger_open(landmarks, a_idx, b_idx,c_idx):
    """指が開いているかを判定"""
    ang = calculate_vector(landmarks[a_idx],landmarks[b_idx],landmarks[c_idx])
    return ang

def oyayubi_finger_open(landmarks, a_idx, b_idx,c_idx):
    """指が開いているかを判定"""
    ang = oyayubi(landmarks[a_idx],landmarks[b_idx],landmarks[c_idx])
    return ang

# カメラキャプチャの開始
cap = cv2.VideoCapture(0)

while True:
    success, image = cap.read()
    if not success:
        print("カメラが検出されませんでした。")
        break

    image = cv2.flip(image, 1)  # 鏡映像のようにする
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(image_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            landmarks = [(lm.x, lm.y) for lm in hand_landmarks.landmark]
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # 指の開閉状態を判定
            thumb_open = oyayubi_finger_open(landmarks, 0,2, 4)  # 親指
            index_open = is_finger_open(landmarks, 0,6, 8)  # 人差し指
            middle_open = is_finger_open(landmarks, 0,10, 12)  # 中指
            ring_open = is_finger_open(landmarks, 0,14, 16)  # 薬指
            pinky_open = is_finger_open(landmarks, 0,18, 20)  # 小指

            # 状態を判定して表示
            if thumb_open and index_open and middle_open and ring_open and pinky_open:
                gesture = "FIVE"
            elif not thumb_open and index_open and middle_open and ring_open and pinky_open:
                gesture = "FOUR"
            elif thumb_open and index_open and middle_open and not ring_open and not pinky_open:
                gesture = "THREE"
            elif thumb_open and index_open and not middle_open and not ring_open and not pinky_open:
                gesture = "TWO"
            elif not thumb_open and index_open and not middle_open and not ring_open and not pinky_open:
                gesture = "ONE"
            else:
                gesture = "UNKNOWN"

            cv2.putText(image, f'Gesture: {gesture}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # 結果を表示
    cv2.imshow("Hand Tracking", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()