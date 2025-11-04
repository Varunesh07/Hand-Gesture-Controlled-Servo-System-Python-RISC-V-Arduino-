extern "C" void set_pwm_duty(int channel, int angle);

void setup() {
  Serial.begin(115200); // Default baud rate, adjust as needed
  Serial.println("Setup complete");
  volatile uint32_t *pwm_base = (volatile uint32_t *)0x10400000;
  pwm_base[0] = 2000000; // Period: 20ms @ 100 MHz
  pwm_base[8] = 1;       // Enable PWM (assumed offset)
  for (int i = 0; i < 5; i++) {
    set_pwm_duty(i, 0);
    Serial.print("Initialized Channel "); Serial.println(i);
  }
}

void loop() {
  if (Serial.available()) {
    String data = "";
    unsigned long start = millis();
    while (millis() - start < 100) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') break;
        data += c;
      }
    }
    if (data.length() == 0) return;
    Serial.print("Received: "); Serial.println(data);

    // Parse the angle data
    int angles[5] = {0, 0, 0, 0, 0}; // T, I, M, R, P
    char dataCopy[64]; // Buffer for the string
    data.toCharArray(dataCopy, sizeof(dataCopy));

    char *token = strtok(dataCopy, ",");
    while (token != NULL) {
      if (strncmp(token, "T:", 2) == 0) {
        angles[0] = atoi(token + 2); // Thumb
      } else if (strncmp(token, "I:", 2) == 0) {
        angles[1] = atoi(token + 2); // Index
      } else if (strncmp(token, "M:", 2) == 0) {
        angles[2] = atoi(token + 2); // Middle
      } else if (strncmp(token, "R:", 2) == 0) {
        angles[3] = atoi(token + 2); // Ring
      } else if (strncmp(token, "P:", 2) == 0) {
        angles[4] = atoi(token + 2); // Pinky
      }
      token = strtok(NULL, ",");
    }

    // Debug output and servo control
    for (int i = 0; i < 5; i++) {
      Serial.print("Channel "); Serial.print(i);
      Serial.print(" Angle: "); Serial.println(angles[i]);
      if (angles[i] >= 0 && angles[i] <= 180) {
        set_pwm_duty(i, angles[i]);
      }
    }
  }
}
/*


// Define servo control pins
#define THUMB_PIN  2
#define INDEX_PIN  3
#define MIDDLE_PIN 4
#define RING_PIN   5
#define PINKY_PIN  6

void setup() {
  Serial.begin(9600); // Start UART

  // Set pins as output
  pinMode(THUMB_PIN, OUTPUT);
  pinMode(INDEX_PIN, OUTPUT);
  pinMode(MIDDLE_PIN, OUTPUT);
  pinMode(RING_PIN, OUTPUT);
  pinMode(PINKY_PIN, OUTPUT);

  Serial.println("Ready to receive angles.");
}

char incomingChar;
String data;

void loop() {
  while (Serial.available()) {
    incomingChar = Serial.read();
    if (incomingChar == '\n') break;
    data += incomingChar;
  }

  if (incomingChar == '\n' && data.length() > 0) {
    int thumb, index, middle, ring, pinky;

    if (sscanf(data.c_str(), "T:%d,I:%d,M:%d,R:%d,P:%d", &thumb, &index, &middle, &ring, &pinky) == 5) {
      thumb = constrain(thumb, 0, 180);
      index = constrain(index, 0, 180);
      middle = constrain(middle, 0, 180);
      ring = constrain(ring, 0, 180);
      pinky = constrain(pinky, 0, 180);

      moveServo(THUMB_PIN, thumb);
      moveServo(INDEX_PIN, index);
      moveServo(MIDDLE_PIN, middle);
      moveServo(RING_PIN, ring);
      moveServo(PINKY_PIN, pinky);

      Serial.println("Servos Moved to Angles.");
    } else {
      Serial.println("Error: Invalid data format.");
    }

    data = "";
  }

  delay(20); // Allow servos to respond
}

// Function to simulate servo PWM signal
void moveServo(int pin, int angle) {
  int pulseWidth = map(angle, 0, 180, 544, 2400); // in microseconds
  for (int i = 0; i < 50; i++) { // ~1 second total
    digitalWrite(pin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(pin, LOW);
    delayMicroseconds(20000 - pulseWidth); // total 20ms per frame
  }
}

*/
/*
#include <Servo.h>
#include <math.h>

// Create servo objects for each finger
Servo thumbServo, indexServo, middleServo, ringServo, pinkyServo;

// Structure to hold x, y coordinates of a landmark
struct Landmark {
  float x;
  float y;
};

// Structure to hold finger angle data
struct FingerAngle {
  const char* name;
  float angle;
};

// Calculate angle between three points (in degrees) using the law of cosines
float calculateAngle(Landmark p1, Landmark p2, Landmark p3) {
  float v1x = p1.x - p2.x;
  float v1y = p1.y - p2.y;
  float v2x = p3.x - p2.x;
  float v2y = p3.y - p2.y;

  float dot = v1x * v2x + v1y * v2y;
  float mag1 = sqrt(v1x * v1x + v1y * v1y);
  float mag2 = sqrt(v2x * v2x + v2y * v2y);

  if (mag1 == 0 || mag2 == 0) return 0.0;

  float cosTheta = dot / (mag1 * mag2);
  if (cosTheta > 1.0) cosTheta = 1.0;
  if (cosTheta < -1.0) cosTheta = -1.0;

  return acos(cosTheta) * 180.0 / M_PI;
}

// Calculate angles for each finger based on landmarks
void getFingerAngles(Landmark* landmarks, FingerAngle* angles) {
  static const int fingerJoints[5][4] = {
    {1, 2, 3, 4},  // Thumb
    {5, 6, 7, 8},  // Index
    {9, 10, 11, 12},  // Middle
    {13, 14, 15, 16},  // Ring
    {17, 18, 19, 20}  // Pinky
  };

  static const char* fingerNames[5] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};

  for (int i = 0; i < 5; i++) {
    angles[i].name = fingerNames[i];
    angles[i].angle = calculateAngle(
      {landmarks[fingerJoints[i][0]].x, landmarks[fingerJoints[i][0]].y},
      {landmarks[fingerJoints[i][1]].x, landmarks[fingerJoints[i][1]].y},
      {landmarks[fingerJoints[i][2]].x, landmarks[fingerJoints[i][2]].y}
    );
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Attach servos to pins 2-6
  thumbServo.attach(2);
  indexServo.attach(3);
  middleServo.attach(4);
  ringServo.attach(5);
  pinkyServo.attach(6);

  Serial.println("Ready to receive landmarks.");
}

char serialBuffer[256];  // Buffer for serial data (fits 42 floats as strings)
int bufferIndex = 0;
bool dataComplete = false;

void loop() {
  // Read serial data non-blocking
  while (Serial.available() && bufferIndex < 255) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuffer[bufferIndex] = '\0';  // Null-terminate
      dataComplete = true;
      break;
    }
    serialBuffer[bufferIndex++] = c;
  }

  if (dataComplete) {
    // Parse landmark coordinates (42 floats: x1,y1,...,x21,y21)
    Landmark landmarks[21];
    int index = 0;
    char* token = strtok(serialBuffer, ",");
    while (token != NULL && index < 42) {
      float value = atof(token);
      if (index % 2 == 0) {
        landmarks[index / 2].x = value;
      } else {
        landmarks[index / 2].y = value;
      }
      index++;
      token = strtok(NULL, ",");
    }

    // Process only if exactly 42 values received
    if (index == 42) {
      FingerAngle angles[5];
      getFingerAngles(landmarks, angles);

      for (int i = 0; i < 5; i++) {
        // Apply thumb adjustment (from original Python code)
        if (i == 0 && angles[i].angle < 125) {
          angles[i].angle -= 120;
          if (angles[i].angle < 0) angles[i].angle = 0;  // Prevent negative angles
        }

        int servoAngle = constrain((int)angles[i].angle, 0, 180);
        switch (i) {
          case 0: thumbServo.write(servoAngle); break;
          case 1: indexServo.write(servoAngle); break;
          case 2: middleServo.write(servoAngle); break;
          case 3: ringServo.write(servoAngle); break;
          case 4: pinkyServo.write(servoAngle); break;
        }
        Serial.print(angles[i].name);
        Serial.print(": ");
        Serial.print(servoAngle);
        Serial.println(" degrees");
      }
      Serial.println("Servos Moved to Angles.");
    } else {
      Serial.print("Error: Invalid landmark data (");
      Serial.print(index);
      Serial.println(" values).");
    }

    // Reset buffer
    bufferIndex = 0;
    dataComplete = false;
    memset(serialBuffer, 0, sizeof(serialBuffer));
  }
}
*/