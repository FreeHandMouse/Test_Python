# Pythonでの開発
## 環境構築
- ライブラリ
  ```
  pip install -r requirements.txt
  ```
- venv
  ```
  python -m venv .venv
  .\.venv\scripts\activate
  ```
## ドキュメント
- [MediaPipe](https://mediapipe.readthedocs.io/en/latest/solutions/hands.html)

## ID対応表

| ID  | 名前       |
|-----|------------|
| 0 | 手首 |
| 2 | 親指-付け根 |
| 4 | 親指-先 |
| 5 | 人差し指-付け根 |
| 8 | 人差し指-先 |
| 9 | 中指-付け根 |
| 12 | 中指-先 |
| 13 | 薬指-付け根 |
| 16 | 薬指-先 |
| 17 | 小指-付け根 |
| 20 | 小指-先 |

![画像](hand_landmarks.png)
