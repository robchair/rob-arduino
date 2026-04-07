// control_chair_bts7960_ultrasonic.ino
// Compatible with BTS7960 drivers with HC-SR04 ultrasonic sensor
// for obstacle detection and automatic stopping.

// -------- PIN DEFINITIONS --------
// Left motor BTS7960
const int L_RPWM = 5;   // Left board RPWM  (PWM)
const int L_LPWM = 6;   // Left board LPWM  (PWM)

// Right motor BTS7960
const int R_RPWM = 9;   // Right board RPWM (PWM)
const int R_LPWM = 10;  // Right board LPWM (PWM)

// HC-SR04 Ultrasonic Sensor
const int TRIG_PIN = 3;
const int ECHO_PIN = 2;

// Optional: if you wired R_EN / L_EN to pins instead of 5V,
// define them here and set them HIGH in setup().
const int L_REN = -1;   // set to real pin if used, otherwise -1
const int L_LEN = -1;
const int R_REN = -1;
const int R_LEN = -1;

// ---- SPEED LIMITS ----
const int MAX_SPEED = 255;    // max PWM
const int FWD_SPEED = 150;    // nominal forward speed
const int TURN_SPEED = 130;   // turning speed
const int BACK_SPEED = 130;   // backward speed

// -------- RAMP SETTINGS --------
const float ACCEL_PWM_PER_SEC = 125.0;   // reach 50 PWM in ~0.4 s
const float DECEL_PWM_PER_SEC = 200.0;   // stop from 50 PWM in ~0.25 s

// -------- MOTOR RAMP STATE --------
float currentLeftPWM = 0.0f;
float currentRightPWM = 0.0f;

int targetLeftPWM = 0;
int targetRightPWM = 0;

unsigned long lastRampMs = 0;

// ---- ULTRASONIC SENSOR SETTINGS ----
const float STOP_DISTANCE_CM = 20.0;  // Stop if obstacle within 20cm
const unsigned long SENSOR_INTERVAL = 100;  // Read sensor every 100ms

// -------- ULTRASONIC SENSOR FUNCTIONS --------
float readDistance() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  
  // Calculate distance in cm
  // Speed of sound = 343 m/s = 0.0343 cm/µs
  // Distance = (duration / 2) * 0.0343
  if (duration == 0) {
    return 999.0;  // No echo received, return large value
  }
  
  float distance = (duration / 2.0) * 0.0343;
  return distance;
}
float moveToward(float current, float target, float maxStep) {
  if (current < target) {
    current += maxStep;
    if (current > target) current = target;
  } else if (current > target) {
    current -= maxStep;
    if (current < target) current = target;
  }
  return current;
}

void requestMotorSpeeds(int leftSpeed, int rightSpeed) {
  targetLeftPWM = clampSpeed(leftSpeed);
  targetRightPWM = clampSpeed(rightSpeed);
}

void applyMotorSpeedsImmediate(int leftSpeed, int rightSpeed) {
  targetLeftPWM = clampSpeed(leftSpeed);
  targetRightPWM = clampSpeed(rightSpeed);
  currentLeftPWM = targetLeftPWM;
  currentRightPWM = targetRightPWM;
  setMotorSpeeds((int)round(currentLeftPWM), (int)round(currentRightPWM));
}

void updateMotorRamp(unsigned long now) {
  if (lastRampMs == 0) {
    lastRampMs = now;
    return;
  }

  float dt = (now - lastRampMs) / 1000.0f;
  lastRampMs = now;

  if (dt <= 0.0f) return;

  float leftRate  = (abs(targetLeftPWM)  < abs(currentLeftPWM))  ? DECEL_PWM_PER_SEC : ACCEL_PWM_PER_SEC;
  float rightRate = (abs(targetRightPWM) < abs(currentRightPWM)) ? DECEL_PWM_PER_SEC : ACCEL_PWM_PER_SEC;

  float leftStep  = leftRate * dt;
  float rightStep = rightRate * dt;

  currentLeftPWM  = moveToward(currentLeftPWM,  (float)targetLeftPWM,  leftStep);
  currentRightPWM = moveToward(currentRightPWM, (float)targetRightPWM, rightStep);

  setMotorSpeeds((int)round(currentLeftPWM), (int)round(currentRightPWM));
}

void waitUntilClear() {
  // Stop motors immediately
  stopMotors();
  Serial.println("OBSTACLE_DETECTED - Waiting for clearance");
  
  // Wait until obstacle is cleared
  while (true) {
    delay(SENSOR_INTERVAL);  // Wait 100ms between checks
    float distance = readDistance();
    
    if (distance >= STOP_DISTANCE_CM) {
      Serial.println("Path clear - Ready");
      break;  // Exit the blocking loop
    }
  }
}

// -------- MOTOR CONTROL HELPERS --------
// Clamp value into [-MAX_SPEED, MAX_SPEED]
int clampSpeed(int s) {
  if (s > MAX_SPEED) return MAX_SPEED;
  if (s < -MAX_SPEED) return -MAX_SPEED;
  return s;
}

// Drive one BTS7960-motor pair with a signed speed:
//   speed > 0  -> forward (RPWM = speed, LPWM = 0)
//   speed < 0  -> backward (RPWM = 0, LPWM = -speed)
//   speed = 0  -> stop (both 0)
void driveBTS7960(int rpwmPin, int lpwmPin, int speed) {
  speed = clampSpeed(speed);
  
  if (speed > 0) {
    analogWrite(rpwmPin, speed);
    analogWrite(lpwmPin, 0);
  } else if (speed < 0) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -speed);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

// Set both motors at once
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  driveBTS7960(L_RPWM, L_LPWM, leftSpeed);
  driveBTS7960(R_RPWM, R_LPWM, rightSpeed);
}

// High-level motion commands
void stopMotors() {
  applyMotorSpeedsImmediate(0, 0);   // keep as hard stop for safety
}

void forward() {
  requestMotorSpeeds(FWD_SPEED, -FWD_SPEED);
}

void backward() {
  requestMotorSpeeds(-BACK_SPEED, BACK_SPEED);
}

void turnLeft() {
  requestMotorSpeeds(0, -TURN_SPEED);
}

void turnRight() {
  requestMotorSpeeds(TURN_SPEED, 0);
}

void quitDrive() {
  applyMotorSpeedsImmediate(0, 0);
}

// -------- ARDUINO SETUP/LOOP --------
void setup() {
  Serial.begin(9600);
  
  // Motor pins
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  
  // Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Enable pins if used
  if (L_REN >= 0) {
    pinMode(L_REN, OUTPUT);
    digitalWrite(L_REN, HIGH);
  }
  if (L_LEN >= 0) {
    pinMode(L_LEN, OUTPUT);
    digitalWrite(L_LEN, HIGH);
  }
  if (R_REN >= 0) {
    pinMode(R_REN, OUTPUT);
    digitalWrite(R_REN, HIGH);
  }
  if (R_LEN >= 0) {
    pinMode(R_LEN, OUTPUT);
    digitalWrite(R_LEN, HIGH);
  }
  
  stopMotors();
  Serial.println("Wheelchair controller with obstacle detection ready");
}

const unsigned long TELEMETRY_INTERVAL_MS = 100;  // publish range every 100ms
const unsigned long CMD_WATCHDOG_MS = 250;        // if no cmd in 250ms -> stop (safety)

unsigned long lastTelemetryMs = 0;
unsigned long lastCmdMs = 0;

void loop() {
  unsigned long now = millis();

  // 1) Read and publish ultrasonic range periodically (telemetry)
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    float distance = readDistance();
    Serial.print("RANGE_CM:");
    Serial.println(distance, 2);   // 2 decimal places
  }

  // 2) Process serial motor commands (Arduino no longer blocks motion)
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    lastCmdMs = now;

    if (command == "forward") {
      forward();
    } else if (command == "backward") {
      backward();
    } else if (command == "turnLeft") {
      turnLeft();
    } else if (command == "turnRight") {
      turnRight();
    } else if (command == "stop") {
      stopMotors();
    } else if (command == "quit") {
      quitDrive();
    }
  }

  // 3) if Jetson stops sending commands, stop motors
  //if ((now - lastCmdMs) > CMD_WATCHDOG_MS) {
    //stopMotors();
  //}
}
