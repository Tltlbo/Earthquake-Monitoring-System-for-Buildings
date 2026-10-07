import numpy as np
import socket
import threading
import time

np.object = object
np.bool = bool
np.complex = complex
np.int = int
np.float = float
np.str = str
np.typeDict = np.sctypeDict

from tensorflow.keras.models import load_model
import cv2


# ==============================
# 서버 설정
# ==============================
SERVER_IP = "10.10.16.74"   # 서버 IP로 수정
SERVER_PORT = 5000

CLIENT_ID = "PJS_JET"
PASSWORD = "PASSWD"


# ==============================
# TCP 서버 연결
# ==============================
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    client_socket.connect((SERVER_IP, SERVER_PORT))
    print(f"[TCP] Server Connected : {SERVER_IP}:{SERVER_PORT}")

    # C 클라이언트와 동일한 로그인 방식
    login_msg = f"[{CLIENT_ID}:{PASSWORD}]"
    client_socket.sendall(login_msg.encode())

    print(f"[TCP] Login message sent : {login_msg}")

except Exception as e:
    print("[TCP] Connection Error:", e)
    exit()


# ==============================
# 서버 수신 Thread
# ==============================
def recv_server():
    while True:
        try:
            data = client_socket.recv(1024)

            if not data:
                print("[TCP] Server disconnected")
                break

            print("[SERVER]", data.decode(errors="ignore"))

        except Exception as e:
            print("[TCP] Receive Error:", e)
            break


recv_thread = threading.Thread(
    target=recv_server,
    daemon=True
)

recv_thread.start()


# ==============================
# AI 모델 설정
# ==============================

# Disable scientific notation for clarity
np.set_printoptions(suppress=True)

# Load the model
model = load_model("keras_model.h5", compile=False)

# Load the labels
class_names = open("labels.txt", "r").readlines()


# ==============================
# Camera
# ==============================

camera = cv2.VideoCapture(0)

if not camera.isOpened():
    print("[CAMERA] Camera open failed")
    client_socket.close()
    exit()


# 이전 AI 판정값
last_class = None


try:

    while True:

        # Webcam 이미지 획득
        ret, image = camera.read()

        if not ret:
            print("[CAMERA] Failed to read frame")
            time.sleep(0.1)
            continue


        # ==============================
        # 모델 입력 이미지 생성
        # ==============================

        image = cv2.resize(
            image,
            (224, 224),
            interpolation=cv2.INTER_AREA
        )


        # ==============================
        # 웹캠 화면 표시 안 함
        # ==============================

        # cv2.imshow("Webcam Image", image)


        # numpy 배열로 변환
        image = np.asarray(
            image,
            dtype=np.float32
        ).reshape(1, 224, 224, 3)


        # Normalize
        image = (image / 127.5) - 1


        # ==============================
        # AI Predict
        # ==============================

        prediction = model.predict(
            image,
            verbose=0
        )

        index = np.argmax(prediction)

        confidence_score = prediction[0][index]


        # labels.txt
        #
        # 0 Warning
        # 1 Caution
        # 2 Safety

        class_name = class_names[index].strip()

        # "0 Warning" → "Warning"
        class_name = class_name.split(
            " ",
            1
        )[1]


        # 서버 전송용 대문자
        current_class = class_name.upper()


        print(
            f"Class: {current_class}, "
            f"Confidence: {confidence_score * 100:.2f}%"
        )


        # ==============================
        # 결과가 변경됐을 때만 서버 전송
        # ==============================

        if current_class != last_class:

            send_msg = f"[PJS_SQL]{current_class}\n"

            try:
                client_socket.sendall(
                    send_msg.encode()
                )

                print(
                    f"[TCP SEND] {send_msg}"
                )

                last_class = current_class

            except Exception as e:
                print(
                    "[TCP] Send Error:",
                    e
                )
                break


        # 화면을 띄우지 않기 때문에
        # ESC 대신 Ctrl+C로 프로그램 종료
        time.sleep(0.05)


except KeyboardInterrupt:

    print("\nProgram terminated by user.")


finally:

    camera.release()

    # imshow를 사용하지 않더라도 호출해도 문제 없음
    cv2.destroyAllWindows()

    client_socket.close()

    print("[TCP] Connection closed")