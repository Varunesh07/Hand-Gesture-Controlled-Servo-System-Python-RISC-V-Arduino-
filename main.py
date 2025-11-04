import cv2
import mediapipe as mp
import numpy as np
import  serial
import math
from collections import Counter
# Initialize serial communication
try:
    ser = serial.Serial('COM3', 9600)  # Update COM Port
    print("Serial Connection Established.")
except Exception as e:
    print(f"Serial Error: {e}")

# Initialize MediaPipe Hand Tracking
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

def calculate_angle(a, b, c):
    """
    Calculate the angle between three points using cosine rule.
    """
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    ab = a - b
    cb = c - b

    dot_product = np.dot(ab, cb)
    magnitude_ab = np.linalg.norm(ab)
    magnitude_cb = np.linalg.norm(cb)

    if magnitude_ab * magnitude_cb == 0:
        return 0

    angle_rad = np.arccos(dot_product / (magnitude_ab * magnitude_cb))
    angle_deg = np.degrees(angle_rad)

    return int(angle_deg)

def get_finger_angles(hand_landmarks):
    """
    Calculate angles for each finger using landmarks.
    Returns a dictionary with finger angles.
    """
    finger_joints = {
        "Thumb": [1, 2, 3, 4],
        "Index": [5, 6, 7, 8],
        "Middle": [9, 10, 11, 12],
        "Ring": [13, 14, 15, 16],
        "Pinky": [17, 18, 19, 20],
    }
    angles = {}

    for finger, joints in finger_joints.items():
        angle = calculate_angle(
            [hand_landmarks.landmark[joints[0]].x, hand_landmarks.landmark[joints[0]].y],
            [hand_landmarks.landmark[joints[1]].x, hand_landmarks.landmark[joints[1]].y],
            [hand_landmarks.landmark[joints[2]].x, hand_landmarks.landmark[joints[2]].y],
        )
        angles[finger] = angle

    return angles

def send_command_to_microcontroller(angles):
    """
    Send finger angles to the RISC-V microcontroller via UART.
    """
    try:
        if angles[0]<125:
            angles[0]-=120
            
        command = f"T:{angles[0]},I:{angles[1]},M:{angles[2]},R:{angles[3]},P:{angles[4]}\n"
    

        ser.write(command.encode())
        print(f"Sent: {command}")
    except Exception as e:
        print(f"Error in sending data: {e}")

# Start Video Capture
cap = cv2.VideoCapture(0)
avg_ang=[]
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            angles = get_finger_angles(hand_landmarks)
            print("Finger Angles:", angles)
             
            avg_ang.append(list(angles.values()))
            if(len(avg_ang)==5):
                counter = Counter(map(tuple, avg_ang))
                common_ang = counter.most_common(1)
                angles = list(common_ang[0][0])
                print(angles)
                send_command_to_microcontroller(angles)

                avg_ang.clear()
