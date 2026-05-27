import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import serial
import time
import math

# --- CONFIGURATION ---
SERIAL_PORT = 'COM9'
BAUD_RATE   = 115200

STANDARD_POS    = [90, 90,  90,  90,  90, 73]
REACH_POS       = [90, 90, 180, 180,  90, 10]
SAFETY_POS      = [180, 90, 90, 90, 90, 30]
LEFT_HAND_JOINTS = [1, 2, 3, 4]

GRIPPER_OPEN   = 73
GRIPPER_CLOSED = 0

MODEL_PATH = "hand_landmarker.task"

FINGER_TIPS = [8, 12, 16, 20]
FINGER_PIPS = [6, 10, 14, 18]

# -------------------------------------------------------------------------
def connect_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        print(f"Connected to {port}")
        return ser
    except serial.SerialException as e:
        print(f"Could not connect to Arduino: {e}")
        exit(1)


def build_landmarker(model_path):
    base_options = python.BaseOptions(model_asset_path=model_path)
    options = vision.HandLandmarkerOptions(
        base_options=base_options,
        num_hands=2,
        min_hand_detection_confidence=0.6,
        min_hand_presence_confidence=0.6,
        min_tracking_confidence=0.5,
        running_mode=vision.RunningMode.VIDEO,
    )
    return vision.HandLandmarker.create_from_options(options)


def finger_closure(lms) -> float:
    """
    Returns closure ratio in [0.0, 1.0].
    0.0 = fully open, 1.0 = fully closed fist.
    """
    curled = sum(
        1 for tip, pip in zip(FINGER_TIPS, FINGER_PIPS)
        if lms[tip].y > lms[pip].y
    )
    return curled / 4.0


def map_range(x, in_lo, in_hi, out_lo, out_hi) -> int:
    x = max(in_lo, min(in_hi, x))
    return int((x - in_lo) / (in_hi - in_lo) * (out_hi - out_lo) + out_lo)


def draw_landmarks(img, lms):
    h, w = img.shape[:2]
    CONNECTIONS = [
        (0,1),(1,2),(2,3),(3,4),
        (0,5),(5,6),(6,7),(7,8),
        (9,10),(10,11),(11,12),
        (13,14),(14,15),(15,16),
        (0,17),(17,18),(18,19),(19,20),
        (5,9),(9,13),(13,17),
    ]
    pts = [(int(lm.x * w), int(lm.y * h)) for lm in lms]
    for a, b in CONNECTIONS:
        cv2.line(img, pts[a], pts[b], (0, 200, 0), 2)
    for pt in pts:
        cv2.circle(img, pt, 4, (0, 255, 0), -1)


# -------------------------------------------------------------------------
def main():
    ser = connect_serial(SERIAL_PORT, BAUD_RATE)
    landmarker = build_landmarker(MODEL_PATH)
    cap = cv2.VideoCapture(0)

    timestamp_ms = 0
    last_command = ""

    try:
        while cap.isOpened():
            ok, img = cap.read()
            if not ok:
                break

            img = cv2.flip(img, 1)
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)

            timestamp_ms += 33
            result = landmarker.detect_for_video(mp_image, timestamp_ms)

            # ---- Parse detections ----------------------------------------
            left_closure  = None
            right_closure = None

            for i, lms in enumerate(result.hand_landmarks):
                label = result.handedness[i][0].category_name
                closure = finger_closure(lms)

                if label == "Left":
                    left_closure = closure
                else:
                    right_closure = closure

                draw_landmarks(img, lms)

            # ---- Compute servo targets ------------------------------------
            m1, m2, m3, m4, m5, m6 = STANDARD_POS

            both_visible = (left_closure is not None and right_closure is not None)
            both_open    = both_visible and left_closure < 0.2 and right_closure < 0.2

            if both_open:
                m1, m2, m3, m4, m5, m6 = STANDARD_POS

            else:
                # Left hand: interpolate arm joints from standard → reach pose
                if left_closure is not None:
                    t = left_closure
                    arm = list(STANDARD_POS)
                    for idx in LEFT_HAND_JOINTS:
                        arm[idx] = round(STANDARD_POS[idx] + t * (REACH_POS[idx] - STANDARD_POS[idx]))
                    m1, m2, m3, m4, m5, m6 = arm

                # Right hand: controls gripper only — overrides m6
                if right_closure is not None:
                    m6 = map_range(right_closure, 0.0, 1.0, GRIPPER_OPEN, GRIPPER_CLOSED)

            # ---- Send command --------------------------------------------
            command = f"{m1},{m2},{m3},{m4},{m5},{m6}\n"
            if command != last_command:
                ser.write(command.encode())
                last_command = command

            # ---- HUD -----------------------------------------------------
            lc_str = f"{left_closure:.2f}"  if left_closure  is not None else "---"
            rc_str = f"{right_closure:.2f}" if right_closure is not None else "---"
            state  = "RESET" if both_open else "CONTROL"

            cv2.putText(img, f"State : {state}",               (10, 35),  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0),     2)
            cv2.putText(img, f"L close: {lc_str}  arm={m2}°",  (10, 70),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180,120,255), 2)
            cv2.putText(img, f"R close: {rc_str}  grip={m6}°", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,140,80),  2)
            cv2.putText(img, f"Cmd: {command.strip()}",         (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(200,200,200), 2)
            cv2.imshow("Braccio Vision Control", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        landmarker.close()
        safety_cmd = ",".join(str(p) for p in SAFETY_POS) + "\n"
        ser.write(safety_cmd.encode())
        time.sleep(1)  # give the arm time to reach the position before port closes
        ser.close()
        

if __name__ == "__main__":
    main()