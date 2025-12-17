import sys
import time
import cv2
import cv2.aruco as aruco

# ===============================
# 경로 설정
# ===============================
sys.path.append('/home/pi/project_demo/lib')
sys.path.append('/home/pi/project_demo')

from Raspbot_Lib import Raspbot
from McLumk_Wheel_Sports import stop_robot

# ===============================
# 로봇 객체
# ===============================
bot = Raspbot()

# ===============================
# 라인트래킹 파라미터
# ===============================
BASE_SPEED  = 3
CURVE_GAIN  = 90
OFFSET_GAIN = 2
LEFT_TRIM   = 0
RIGHT_TRIM  = 0

# ===============================
# 이동 / 회피
# ===============================
OBSTACLE_DIST = 100

BACK_SPEED  = -30
TURN_SPEED  = 35
BACK_TIME   = 0.25
TURN_TIME   = 0.6

FORWARD_SPEED = 10
FORWARD_TIME  = 0.7

# ===============================
# AprilTag 설정
# ===============================
DICT_TYPE = aruco.DICT_APRILTAG_36h11
VALID_TAG_IDS = [10, 20]
CHECK_TIME = 2.0

# ===============================
# 카메라 서보 설정
# ===============================
H_SERVO_ID = 1
V_SERVO_ID = 2

H_MID = 100
V_MID = 10

SERVO_STEP  = 15
SERVO_DELAY = 0.4

# ===============================
# 모터
# ===============================
def set_motors(lf, lr, rf, rr):
    bot.Ctrl_Muto(0, lf + LEFT_TRIM)
    bot.Ctrl_Muto(1, lr + LEFT_TRIM)
    bot.Ctrl_Muto(2, rf + RIGHT_TRIM)
    bot.Ctrl_Muto(3, rr + RIGHT_TRIM)

# ===============================
# 부저
# ===============================
def beep(times=1, duration=0.25, gap=0.2):
    for _ in range(times):
        bot.Ctrl_BEEP_Switch(1)
        time.sleep(duration)
        bot.Ctrl_BEEP_Switch(0)
        time.sleep(gap)

# ===============================
# 초음파
# ===============================
def read_ultrasonic():
    h = bot.read_data_array(0x1b, 1)
    l = bot.read_data_array(0x1a, 1)
    if h is None or l is None:
        return 9999
    return (h[0] << 8) | l[0]

# ===============================
# 라인트래킹 (1 step)
# ===============================
def line_tracking():
    data = bot.read_data_array(0x0a, 1)
    if data is None:
        return False

    v = data[0]
    p1 = (v >> 2) & 1
    p2 = (v >> 3) & 1
    p3 = (v >> 1) & 1
    p4 = v & 1

    L = p1 + p2
    R = p3 + p4

    if p2 or p3:
        corr = int((R - L) * OFFSET_GAIN)
        set_motors(BASE_SPEED-corr, BASE_SPEED-corr,
                   BASE_SPEED+corr, BASE_SPEED+corr)
        return False

    if p4:
        k = min(1.0, (R-L)*0.7)
        s = 2
        set_motors(s+int(CURVE_GAIN*1.1*k), s+int(CURVE_GAIN*1.1*k),
                   s-int(CURVE_GAIN*0.6*k), s-int(CURVE_GAIN*0.6*k))
        return False

    if p1:
        k = min(1.0, (L-R)*0.7)
        s = 2
        set_motors(s-int(CURVE_GAIN*0.6*k), s-int(CURVE_GAIN*0.6*k),
                   s+int(CURVE_GAIN*1.1*k), s+int(CURVE_GAIN*1.1*k))
        return False

    if L == 0 and R == 0:
        stop_robot()
        print("=== CHECKPOINT ===")
        return True

    set_motors(BASE_SPEED, BASE_SPEED, BASE_SPEED, BASE_SPEED)
    return False

# ===============================
# 장애물 회피
# ===============================
def avoid_obstacle():
    set_motors(BACK_SPEED, BACK_SPEED, BACK_SPEED, BACK_SPEED)
    time.sleep(BACK_TIME)
    stop_robot()

    set_motors(TURN_SPEED, TURN_SPEED, -TURN_SPEED, -TURN_SPEED)
    time.sleep(TURN_TIME)
    stop_robot()

# ===============================
# 카메라 sweep
# ===============================
def sweep_horizontal(start, end, detected_ids, cap):
    step = SERVO_STEP if end > start else -SERVO_STEP
    angle = start

    while (angle <= end if step > 0 else angle >= end):
        bot.Ctrl_Servo(H_SERVO_ID, angle)
        time.sleep(SERVO_DELAY)
        detect_tag_step(cap, detected_ids)
        angle += step

def cam_center():
    bot.Ctrl_Servo(H_SERVO_ID, H_MID)
    bot.Ctrl_Servo(V_SERVO_ID, V_MID)
    time.sleep(SERVO_DELAY)

# ===============================
# AprilTag 한 스텝 감지
# ===============================
def detect_tag_step(cap, detected_ids, dwell_time=0.4):
    start = time.time()
    
    while time.time() - start < dwell_time:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(DICT_TYPE)
        params = aruco.DetectorParameters()

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
        if ids is None:
            continue

        for c, i in zip(corners, ids):
            tag_id = int(i[0])
            if tag_id not in VALID_TAG_IDS:
                continue
            if tag_id in detected_ids:
                continue

            if tag_id == 10:
                print("This is our force!")
                beep(1)
            elif tag_id == 20:
                print("Enemy detected!")
                beep(3)

            detected_ids.add(tag_id)
        
            time.sleep(1.0)
            return

# ===============================
# 카메라 전체 스캔
# ===============================
def camera_scan_for_apriltag():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    detected_ids = set()

    cam_center()
    sweep_horizontal(H_MID, H_MID + 60, detected_ids, cap)
    sweep_horizontal(H_MID + 60, H_MID - 60, detected_ids, cap)

    cam_center()
    cap.release()

# ===============================
# 앞으로 10cm
# ===============================
def move_forward_10cm():
    set_motors(FORWARD_SPEED, FORWARD_SPEED,
               FORWARD_SPEED, FORWARD_SPEED)
    time.sleep(FORWARD_TIME)
    stop_robot()

# ===============================
# 메인 루프
# ===============================
def main():
    bot.Ctrl_Ulatist_Switch(1)
    time.sleep(0.1)

    while True:
        checkpoint = line_tracking()

        if checkpoint:
            stop_robot()
            time.sleep(1)

            if read_ultrasonic() <= OBSTACLE_DIST:
                avoid_obstacle()
            else:
                camera_scan_for_apriltag()
                move_forward_10cm()

        time.sleep(0.01)

# ===============================
# 실행
# ===============================
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        stop_robot()
        bot.Ctrl_Ulatist_Switch(0)
        print("SYSTEM STOPPED")
