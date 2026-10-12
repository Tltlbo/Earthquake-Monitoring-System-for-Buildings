
import numpy as np
import socket
import threading
import time

# ============================================================
# NumPy 호환성 설정
# ============================================================

import warnings

with warnings.catch_warnings():
    warnings.simplefilter("ignore", FutureWarning)

    if "object" not in np.__dict__:
        np.object = object

    if "bool" not in np.__dict__:
        np.bool = bool

    if "complex" not in np.__dict__:
        np.complex = complex

    if "int" not in np.__dict__:
        np.int = int

    if "float" not in np.__dict__:
        np.float = float

    if "str" not in np.__dict__:
        np.str = str

    if "typeDict" not in np.__dict__:
        np.typeDict = np.sctypeDict


from tensorflow.keras.models import load_model
from PIL import Image, ImageOps
import cv2


# ============================================================
# TCP 서버 설정
# ============================================================

SERVER_IP = "10.10.16.74"
SERVER_PORT = 5000

LOGIN_ID = "PJS_JET"
CLIENT_ID = "PJS_SQL"
PASSWORD = "PASSWD"


# ============================================================
# AI 판정 안정화 설정
# ============================================================

# 동일 상태가 연속으로 나와야 하는 프레임 수
STABLE_FRAME_COUNT = 5

# 최소 신뢰도 (0.0 = 제한 없음)
MIN_CONFIDENCE = 0.0

# 서버에 상태 재전송하는 주기
HEARTBEAT_INTERVAL = 1.0

# 추론 사이 추가 지연
FRAME_DELAY = 0.03

# 전체 클래스별 확률 출력 여부
PRINT_ALL_PREDICTIONS = True


# ============================================================
# 모델 설정
# ============================================================

MODEL_PATH = "keras_model.h5"
LABEL_PATH = "labels.txt"

MODEL_WIDTH = 224
MODEL_HEIGHT = 224


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

    login_msg = f"[{LOGIN_ID}:{PASSWORD}]"

    client_socket.sendall(
        login_msg.encode("utf-8")
    )

    print(
        f"[TCP] Login Sent : {login_msg}"
    )

except Exception as e:

    print("[TCP] Connection Error:", e)

    client_socket.close()

    raise SystemExit(1)


# ============================================================
# 서버 수신 Thread
# ============================================================

def recv_server():

    while True:

        try:

            data = client_socket.recv(1024)

            if not data:

                print("[TCP] Server disconnected")
                break

            print(
                "[SERVER]",
                data.decode(
                    "utf-8",
                    errors="ignore"
                )
            )

        except Exception as e:

            print("[TCP] Receive Error:", e)
            break


recv_thread = threading.Thread(
    target=recv_server,
    daemon=True
)

recv_thread.start()


# ============================================================
# AI 모델 로드
# ============================================================

np.set_printoptions(suppress=True)

model = load_model(
    MODEL_PATH,
    compile=False
)

with open(
    LABEL_PATH,
    "r",
    encoding="utf-8"
) as file:

    class_names = []

    for line in file:

        label = line.strip()

        if not label:
            continue

        parts = label.split(" ", 1)

        if len(parts) == 2:
            label = parts[1]

        class_names.append(label.upper())


print("================================")
print("[AI] Model Loaded")
print("[AI] Input Shape:", model.input_shape)
print("[AI] Output Shape:", model.output_shape)
print("[AI] Classes:", class_names)
print("================================")

if model.input_shape[1:3] != (
    MODEL_HEIGHT,
    MODEL_WIDTH
):
    raise ValueError(
        "Model input size is not 224x224"
    )

if model.output_shape[-1] != len(class_names):
    raise ValueError(
        "Model output count and label count differ"
    )


# ============================================================
# Pillow 버전 호환
# ============================================================

try:
    RESAMPLE_METHOD = Image.Resampling.LANCZOS

except AttributeError:
    RESAMPLE_METHOD = Image.LANCZOS


# ============================================================
# Teachable Machine 이미지 전처리
# ============================================================

def preprocess_image(frame):

    # OpenCV BGR -> RGB
    rgb_image = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2RGB
    )

    # NumPy -> PIL
    pil_image = Image.fromarray(
        rgb_image
    )

    # 중앙 크롭 + 224x224
    pil_image = ImageOps.fit(
        pil_image,
        (MODEL_WIDTH, MODEL_HEIGHT),
        method=RESAMPLE_METHOD,
        centering=(0.5, 0.5)
    )

    # PIL -> NumPy
    image_array = np.asarray(
        pil_image,
        dtype=np.float32
    )

    # -1 ~ 1 정규화
    normalized_image = (
        image_array / 127.5
    ) - 1.0

    # (224,224,3) -> (1,224,224,3)
    model_input = np.expand_dims(
        normalized_image,
        axis=0
    )

    return model_input


# ============================================================
# Camera
# ============================================================

camera = cv2.VideoCapture(0)

if not camera.isOpened():

    print("[CAMERA] Camera open failed")

    client_socket.close()

    raise SystemExit(1)


print("[CAMERA] Camera Opened")
print("[CAMERA] Display Disabled")


# ============================================================
# AI 상태 변수
# ============================================================

last_prediction = None

same_count = 0

confirmed_class = None

last_send_time = 0.0


# ============================================================
# Main Loop
# ============================================================

try:

    while True:

        # ----------------------------------------------------
        # 카메라 이미지 획득
        # ----------------------------------------------------

        ret, frame = camera.read()

        if not ret:

            print("[CAMERA] Failed to read frame")

            time.sleep(0.1)

            continue


        # ----------------------------------------------------
        # Teachable Machine 전처리
        # ----------------------------------------------------

        model_input = preprocess_image(
            frame
        )


        # ----------------------------------------------------
        # AI Predict
        # ----------------------------------------------------

        prediction = model.predict(
            model_input,
            verbose=0
        )

        scores = prediction[0]

        index = int(
            np.argmax(scores)
        )

        confidence_score = float(
            scores[index]
        )

        current_class = class_names[index]


        # ----------------------------------------------------
        # 클래스별 확률 출력
        # ----------------------------------------------------

        if PRINT_ALL_PREDICTIONS:

            print("------------------------------")

            for i, score in enumerate(scores):

                print(
                    f"[AI] {class_names[i]}: "
                    f"{score * 100:.2f}%"
                )


        # ----------------------------------------------------
        # 최소 신뢰도 검사
        # ----------------------------------------------------

        if confidence_score < MIN_CONFIDENCE:

            print(
                f"[AI] Low Confidence: "
                f"{confidence_score * 100:.1f}%"
            )

            last_prediction = None
            same_count = 0

            time.sleep(FRAME_DELAY)

            continue


        # ----------------------------------------------------
        # 동일 상태 연속 판정
        # ----------------------------------------------------

        if current_class == last_prediction:

            same_count += 1

        else:

            last_prediction = current_class
            same_count = 1


        # ----------------------------------------------------
        # AI 상태 출력
        # ----------------------------------------------------

        print(
            f"[AI] {current_class} "
            f"{confidence_score * 100:.1f}% "
            f"| stable={same_count}"
        )


        # ====================================================
        # 안정화된 상태만 TCP 전송
        # ====================================================

        current_time = time.monotonic()

        if same_count >= STABLE_FRAME_COUNT:

            # 상태 변경 여부
            state_changed = (
                current_class != confirmed_class
            )

            # Heartbeat 주기 도달 여부
            heartbeat_due = (
                current_time - last_send_time
                >= HEARTBEAT_INTERVAL
            )

            if state_changed or heartbeat_due:

                # --------------------------------------------
                # TCP 패킷 생성
                # --------------------------------------------

                send_msg = (
                    f"[{CLIENT_ID}]"
                    f"STATUS@{current_class}\n"
                )

                try:

                    client_socket.sendall(
                        send_msg.encode("utf-8")
                    )

                    # ----------------------------------------
                    # 상태 변경 로그
                    # ----------------------------------------

                    if state_changed:

                        print(
                            "================================"
                        )

                        print(
                            "[TCP SEND (STATE CHANGE)]",
                            send_msg.strip()
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

                    # ----------------------------------------
                    # Heartbeat 로그
                    # ----------------------------------------

                    else:

                        print(
                            "[TCP HEARTBEAT]",
                            send_msg.strip()
                        )


                    # ----------------------------------------
                    # 전송 상태 저장
                    # ----------------------------------------

                    confirmed_class = current_class

                    last_send_time = current_time


                except Exception as e:

                    print("[TCP] Send Error:", e)

                    break


        # ----------------------------------------------------
        # CPU 부하 완화
        # ----------------------------------------------------

        time.sleep(FRAME_DELAY)


# ============================================================
# Ctrl + C 종료
# ============================================================

except KeyboardInterrupt:

    print("\n[PROGRAM] User terminated")


# ============================================================
# 종료 처리
# ============================================================

finally:

    camera.release()

    # GUI를 사용하지 않으므로
    # cv2.destroyAllWindows() 호출하지 않음

    try:

        client_socket.shutdown(
            socket.SHUT_RDWR
        )

    except OSError:
        pass

    client_socket.close()

    print("[TCP] Connection closed")
