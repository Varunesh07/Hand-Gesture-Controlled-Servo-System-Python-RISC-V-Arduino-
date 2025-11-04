# ✋ Hand Gesture Controlled Servo System (Python + RISC-V + Arduino)

## 🧩 Overview
This project uses **MediaPipe** and **OpenCV** to track hand gestures in real time and send corresponding **finger angles** over a serial connection to a **RISC-V microcontroller**.  
The microcontroller, programmed using both **Arduino C** and **RISC-V assembly**, drives **PWM signals** to control servo motors based on detected finger movements.

---

## ⚙️ System Architecture

**Python (Frontend)**  
→ captures video via webcam, tracks hand landmarks, calculates finger angles, and transmits data (UART).  

**Microcontroller (Backend)**  
→ receives serial commands, decodes them, and updates servo PWM outputs through low-level RISC-V assembly.

```
Camera → Python (MediaPipe) → Serial (UART) → Microcontroller → Servo Motors
```

---

## 🧠 Components

| Component | Description |
|------------|-------------|
| **Python** | Captures hand landmarks using MediaPipe and sends angles to the microcontroller |
| **MediaPipe** | Hand tracking (detects 21 keypoints) |
| **OpenCV** | Webcam frame capture and visualization |
| **NumPy** | Vector and angle calculations |
| **Serial (pySerial)** | UART communication with the microcontroller |
| **RISC-V Assembly (pwm.S)** | Sets PWM duty cycle for servos |
| **Arduino Sketch (processingfile.ino)** | Handles serial input and maps received angles to PWM output pins |

---

## 🧮 Python Workflow

1. Captures live video frames (`cv2.VideoCapture`).
2. Detects hand landmarks using **MediaPipe Hands**.
3. Computes **finger bending angles** using a cosine-rule based function.
4. Averages readings over 5 frames for stability.
5. Sends formatted UART command:
   ```
   T:<thumb_angle>,I:<index_angle>,M:<middle_angle>,R:<ring_angle>,P:<pinky_angle>
   ```
6. Microcontroller receives and adjusts servo positions accordingly.

---

## 🧱 Microcontroller Logic

### `processingfile.ino`
- Reads serial data from Python.
- Parses the incoming string to extract individual finger angles.
- Sends these angles to PWM driver routines (RISC-V).

### `pwm.S`
- Implements `set_pwm_duty` function:
  - Calculates PWM duty cycle using:
    ```
    Duty = 50,000 + (angle × 200,000) / 180
    ```
  - Maps angles (0°–180°) to 0.5 ms–2.5 ms duty range (standard servo control).
  - Writes the computed value to memory-mapped PWM register:
    ```
    Base Address: 0x10400000
    Offset: channel × 4
    ```
  - Updates the correct PWM channel with new duty cycle.

---

## 🛠️ Setup Instructions

### 🧩 Python Side
1. Install dependencies:
   ```bash
   pip install opencv-python mediapipe numpy pyserial
   ```
2. Update serial port in the Python file:
   ```python
   ser = serial.Serial('COM3', 9600)
   ```
   Change `'COM3'` to your actual port.

3. Run the Python script:
   ```bash
   python gesture_control.py
   ```

---

### ⚙️ Microcontroller Side
1. Flash the `processingfile.ino` sketch to your RISC-V/Arduino-compatible board.
2. Assemble and link `pwm.S` using the RISC-V toolchain:
   ```bash
   riscv64-unknown-elf-as pwm.S -o pwm.o
   riscv64-unknown-elf-ld pwm.o -o pwm.elf
   ```
3. Ensure UART baud rate = 9600 bps and connect the board to your PC.

---

## 🧪 Testing
- Launch the Python script.  
- Show your hand to the webcam.  
- Observe angles printed in the console and sent to the microcontroller.  
- Corresponding servo motors should move according to finger positions.

---

## 👨‍💻 Contributors
<table>
  <tr>
     <td align="center"> <a href="https://github.com/NithiishSD"> <img src="https://avatars.githubusercontent.com/u/178805412?v=4" width="100px;" alt="Nithiish SD"/> <br /> <sub><b>Nithiish SD</b></sub> </a> </td>
    <td align="center">
      <a href="https://github.com/Varunesh07">
        <img src="https://avatars.githubusercontent.com/u/205139899?v=4" width="100px;" alt="Varunesh S"/>
        <br />
        <sub><b>Varunesh S</b></sub>
      </a>
    </td>
    <td align="center"> <a href="https://github.com/Monarch0703"> <img src="https://avatars.githubusercontent.com/u/118116807?v=4" width="100px;" alt="Harshith Shiva"/> <br /> <sub><b>Jithendra U</b></sub> </a> </td> 
  </tr>
</table>

---

## 🏁 License
This project is created for academic and prototyping purposes.  
You may reuse or modify it with attribution.
