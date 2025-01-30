import math

# 距離計算関数
def distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

def sgn(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0
    
def angle_3d(vector1, vector2):
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
    angle = angle_3d(atob, btoc)
    return angle