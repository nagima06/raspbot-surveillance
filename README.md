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


## Demo Video
A demonstration video will be provided showing the following steps:
1. Input: Camera capturing the environment.
2. Inference: Vision-based detection process.
3. Output: Physical robot actions such as movement, camera rotation, or sound output.

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

A demonstration of the autonomous surveillance robot is available at the link below.

🎥 Demo video:
- https://youtu.be/XXXXXXX
















