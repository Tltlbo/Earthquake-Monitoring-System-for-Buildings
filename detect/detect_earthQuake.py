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


# ============================================================
# TCP 서버 설정
# ============================================================

SERVER_IP = "10.10.16.74"   # 실제 서버 IP에 맞게 수정
SERVER_PORT = 5000

CLIENT_ID = "PJS_SQL"
PASSWORD = "PASSWD"


# ============================================================
# AI 판정 안정화 설정
# ============================================================

# 같은 결과가 몇 프레임 연속 나와야 확정할지
STABLE_FRAME_COUNT = 5

# 너무 낮은 신뢰도의 결과는 무시하고 싶으면 사용
# 예: 0.70 = 70%
MIN_CONFIDENCE = 0.0


# ============================================================
# TCP 서버 연결
# ============================================================

client_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM
)

try:
    client_socket.connect(
        (SERVER_IP, SERVER_PORT)
    )

    print(
        f"[TCP] Server Connected : "
        f"{SERVER_IP}:{SERVER_PORT}"
    )

    # 기존 C 클라이언트와 동일한 로그인 방식
    login_msg = f"[{CLIENT_ID}:{PASSWORD}]"

    client_socket.sendall(
        login_msg.encode()
    )

    print(
        f"[TCP] Login Sent : "
        f"{login_msg}"
    )

except Exception as e:
    print(
        "[TCP] Connection Error:",
        e
    )

    exit()


# ============================================================
# 서버 수신 Thread
# ============================================================

def recv_server():

    while True:

        try:

            data = client_socket.recv(1024)

            if not data:
                print(
                    "[TCP] Server disconnected"
                )
                break

            print(
                "[SERVER]",
                data.decode(
                    errors="ignore"
                )
            )

        except Exception as e:

            print(
                "[TCP] Receive Error:",
                e
            )

            break


recv_thread = threading.Thread(
    target=recv_server,
    daemon=True
)

recv_thread.start()


# ============================================================
# AI 모델 로드
# ============================================================

np.set_printoptions(
    suppress=True
)

model = load_model(
    "keras_model.h5",
    compile=False
)

class_names = open(
    "labels.txt",
    "r"
).readlines()


# ============================================================
# Camera
# ============================================================

camera = cv2.VideoCapture(0)

if not camera.isOpened():

    print(
        "[CAMERA] Camera open failed"
    )

    client_socket.close()

    exit()


# ============================================================
# AI 상태 변수
# ============================================================

# 바로 이전 프레임의 AI 판정
last_prediction = None

# 같은 판정이 연속으로 나온 횟수
same_count = 0

# 서버에 마지막으로 전송한 확정 상태
confirmed_class = None


# ============================================================
# Main Loop
# ============================================================

try:

    while True:

        # ----------------------------------------------------
        # 카메라 이미지 획득
        # ----------------------------------------------------

        ret, image = camera.read()

        if not ret:

            print(
                "[CAMERA] Failed to read frame"
            )

            time.sleep(0.1)

            continue


        # ----------------------------------------------------
        # 모델 입력 크기로 변경
        # ----------------------------------------------------

        image = cv2.resize(
            image,
            (224, 224),
            interpolation=cv2.INTER_AREA
        )


        # ----------------------------------------------------
        # 웹캠 화면 표시 안 함
        # ----------------------------------------------------

        # cv2.imshow(
        #     "Webcam Image",
        #     image
        # )


        # ----------------------------------------------------
        # numpy array 변환
        # ----------------------------------------------------

        image = np.asarray(
            image,
            dtype=np.float32
        ).reshape(
            1,
            224,
            224,
            3
        )


        # ----------------------------------------------------
        # Normalize
        # ----------------------------------------------------

        image = (
            image / 127.5
        ) - 1


        # ----------------------------------------------------
        # AI Predict
        # ----------------------------------------------------

        prediction = model.predict(
            image,
            verbose=0
        )

        index = np.argmax(
            prediction
        )

        confidence_score = (
            prediction[0][index]
        )


        # ----------------------------------------------------
        # labels.txt
        #
        # 0 Warning
        # 1 Caution
        # 2 Safety
        # ----------------------------------------------------

        class_name = (
            class_names[index]
            .strip()
        )

        # "0 Warning"
        #      ↓
        # "Warning"

        parts = class_name.split(
            " ",
            1
        )

        if len(parts) == 2:

            class_name = parts[1]

        current_class = (
            class_name.upper()
        )


        # ----------------------------------------------------
        # 현재 AI 결과 출력
        # ----------------------------------------------------

        print(
            f"[AI] "
            f"{current_class} "
            f"{confidence_score * 100:.1f}% "
            f"| stable={same_count}"
        )


        # ----------------------------------------------------
        # 신뢰도가 너무 낮으면 이번 결과 무시
        # ----------------------------------------------------

        if confidence_score < MIN_CONFIDENCE:

            last_prediction = None
            same_count = 0

            continue


        # ====================================================
        # 같은 판정이 연속으로 나오는지 검사
        # ====================================================

        if current_class == last_prediction:

            same_count += 1

        else:

            # 다른 판정이 나오면
            # 새 상태로 카운트 시작

            last_prediction = current_class

            same_count = 1


        # ====================================================
        # 일정 프레임 이상 같은 결과가 나온 경우
        # ====================================================

        if same_count >= STABLE_FRAME_COUNT:

            # 이미 서버에 보낸 상태라면
            # 다시 보내지 않음

            if current_class != confirmed_class:

                # TCP 메시지 끝에 \n 추가
                #
                # 예:
                # [PJS_JET]WARNING\n

                send_msg = (
                    f"[{CLIENT_ID}]"
                    f"{current_class}\n"
                )

                try:

                    client_socket.sendall(
                        send_msg.encode()
                    )

                    print(
                        "================================"
                    )

                    print(
                        f"[TCP SEND] "
                        f"{send_msg.strip()}"
                    )

                    print(
                        f"[CONFIDENCE] "
                        f"{confidence_score * 100:.1f}%"
                    )

                    print(
                        f"[STABLE FRAME] "
                        f"{same_count}"
                    )

                    print(
                        "================================"
                    )

                    # 서버에 전송한 상태 저장
                    confirmed_class = (
                        current_class
                    )

                except Exception as e:

                    print(
                        "[TCP] Send Error:",
                        e
                    )

                    break


        # CPU 과부하 방지
        time.sleep(0.03)


# ============================================================
# Ctrl + C 종료
# ============================================================

except KeyboardInterrupt:

    print(
        "\n[PROGRAM] User terminated"
    )


# ============================================================
# 종료 처리
# ============================================================

finally:

    camera.release()

    cv2.destroyAllWindows()

    try:

        client_socket.close()

    except:
        pass

    print(
        "[TCP] Connection closed"
    )