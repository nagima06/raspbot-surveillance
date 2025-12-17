# raspbot-surveillance

# Autonomous Raspberry-Bot with Vision-Based Detection

## Project Overview
This project presents an autonomous Raspberry Pi-based robot designed for surveillance
and target detection tasks. The robot follows a predefined line path and stops at specific
checkpoints to perform vision-based detection using a camera. Based on the detection result,
the system triggers physical responses such as motor control, camera movement, and sound output.
The goal of this project is to demonstrate how AI inference can be integrated with real-world
robotic systems.

## Deployment Track
This project follows **Track B: Raspberry Pi Physical Computing**.
The system operates in a real-world environment by integrating camera-based vision detection,
sensor input, and physical robot actions such as movement, camera rotation, and audio output.

## System Architecture
The system follows an input–inference–output pipeline.
Visual input is captured through the camera and processed using vision-based detection logic.
The inference result is then translated into physical actions such as robot movement,
camera rotation via servo motors, and audio feedback through a speaker.

## Hardware Setup
The robot is built using a Raspberry Pi as the main controller.
The system includes a camera module for vision-based detection,
DC motors for movement, ultrasonic sensors for obstacle awareness,
servo motors for camera rotation, and a speaker for audio feedback.

All hardware testing was conducted carefully in the Makerspace.
The robot remains undamaged and fully functional.


## How to Run
1. Power on the Raspberry Pi and connect all sensors and the camera.
2. Place the robot on the line path.
3. Start the main control program on the Raspberry Pi.
4. The robot begins line tracking and stops at checkpoints.
5. Vision-based detection is performed and physical actions are triggered.

## Error Board

**Case 1: Detection failure under low lighting**
- Hypothesis: Low contrast between the target and background
- Action: Adjusted camera angle and lighting conditions

**Case 2: Camera vibration during movement**
- Hypothesis: Motor vibration affected camera stability
- Action: Reduced movement speed at checkpoints

## Limitations & Future Work
This system relies on rule-based logic rather than a learning-based model,
which limits adaptability to unfamiliar environments.
Performance may vary depending on lighting conditions and physical setup.
Future work includes integrating a learning-based model and improving
robustness under diverse real-world scenarios.






## Full Source Code

The complete control program for the autonomous Raspbot is provided below.

📄 Main control code:  
- [`raspbot_main.py`](code/raspbot_main.py)

This file contains the full implementation of navigation, obstacle avoidance,
camera control, and AprilTag-based detection.


## Demo Video
A demonstration video will be provided showing the following steps:
1. Input: Camera capturing the environment.
2. Inference: Vision-based detection process.
3. Output: Physical robot actions such as movement, camera rotation, or sound output.

A demonstration of the autonomous surveillance robot is available at the link below.

🎥 Demo video:
- https://youtube.com/shorts/Eiyitz5DIfM?feature=share


---

## Code Architecture Breakdown

Below, each functional part of the final control code is shown with a short explanation of its role
in the autonomous system. Code snippets are followed by plain-language descriptions for clarity.

---

## 1. System Setup and Initialization

```python
import sys
import time
import cv2
import cv2.aruco as aruco

sys.path.append('/home/pi/project_demo/lib')
sys.path.append('/home/pi/project_demo')

from Raspbot_Lib import Raspbot
from McLumk_Wheel_Sports import stop_robot

bot = Raspbot()

```
This section initializes all required libraries and creates the main robot control object.
It prepares the system for hardware control, sensor access, and vision processing.



## 2. Line Tracking Parameters

```python
BASE_SPEED  = 3
CURVE_GAIN  = 90
OFFSET_GAIN = 2
LEFT_TRIM   = 0
RIGHT_TRIM  = 0


```
This section initializes all required libraries and creates the main robot control object.
It prepares the system for hardware control, sensor access, and vision processing.


## 3. Obstacle Detection Parameters

```python
OBSTACLE_DIST = 100

BACK_SPEED  = -30
TURN_SPEED  = 35
BACK_TIME   = 0.25
TURN_TIME   = 0.6


```
This configuration defines the distance threshold for obstacle detection and
the motion profile used during avoidance maneuvers.


## 4. Motor Control Function

```python
def set_motors(lf, lr, rf, rr):
    bot.Ctrl_Muto(0, lf + LEFT_TRIM)
    bot.Ctrl_Muto(1, lr + LEFT_TRIM)
    bot.Ctrl_Muto(2, rf + RIGHT_TRIM)
    bot.Ctrl_Muto(3, rr + RIGHT_TRIM)

```
This function provides centralized control of all four motors.
Higher-level behaviors call this function to execute movement commands consistently.


## 5. Line Tracking and Checkpoint Detection

```python
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

    if L == 0 and R == 0:
        stop_robot()
        return True

    return False

```
This function controls autonomous line following.
When all line sensors lose detection, the robot interprets the position as a checkpoint
and pauses for decision making.


## 6. Ultrasonic Distance Measurement


```python
def read_ultrasonic():
    h = bot.read_data_array(0x1b, 1)
    l = bot.read_data_array(0x1a, 1)
    if h is None or l is None:
        return 9999
    return (h[0] << 8) | l[0]

```
This function measures the distance to obstacles in front of the robot using an ultrasonic sensor.
The returned value is used to decide whether avoidance behavior is required.

## 7. Obstacle Avoidance Behavior

```python
def avoid_obstacle():
    set_motors(BACK_SPEED, BACK_SPEED, BACK_SPEED, BACK_SPEED)
    time.sleep(BACK_TIME)
    stop_robot()

    set_motors(TURN_SPEED, TURN_SPEED, -TURN_SPEED, -TURN_SPEED)
    time.sleep(TURN_TIME)
    stop_robot()

```
When an obstacle is detected, the robot reverses briefly and then rotates.
This simple but reliable maneuver prevents collisions in real environments.


## 8. Camera Servo Control and Scanning

```python
def sweep_horizontal(start, end, detected_ids, cap):
    step = SERVO_STEP if end > start else -SERVO_STEP
    angle = start

    while (angle <= end if step > 0 else angle >= end):
        bot.Ctrl_Servo(H_SERVO_ID, angle)
        time.sleep(SERVO_DELAY)
        detect_tag_step(cap, detected_ids)
        angle += step

```
This function moves the camera horizontally using servo motors.
By sweeping the camera, the robot expands its visual field for target detection.

## 9. AprilTag Detection and Classification


```python
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

            if tag_id == 10:
                beep(1)
            elif tag_id == 20:
                beep(3)

            detected_ids.add(tag_id)
            return

```
This vision module detects AprilTags and classifies them by ID.
Different audio signals are used to distinguish friendly and enemy targets.

## 10. Main Decision Loop


```python
def main():
    while True:
        checkpoint = line_tracking()

        if checkpoint:
            if read_ultrasonic() <= OBSTACLE_DIST:
                avoid_obstacle()
            else:
                camera_scan_for_apriltag()
                move_forward_10cm()

```
The main loop integrates all modules into a single autonomous workflow.
The robot follows a repeated cycle of navigation, checkpoint recognition,
obstacle evaluation, visual scanning, and continued movement without human intervention.






















