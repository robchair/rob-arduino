// control_chair_bts7960_ultrasonic_encoder.ino
// BTS7960 motor controller + HC-SR04 + quadrature encoders
// Single Arduino Mega serial interface for motor commands + encoder telemetry

// -------- PIN DEFINITIONS --------
// Left motor BTS7960
const int L_RPWM = 5;
const int L_LPWM = 6;


// Right motor BTS7960
const int R_RPWM = 9;
const int R_LPWM = 10;

// HC-SR04 Ultrasonic Sensor
// Moved off pins 2 and 3 to avoid encoder interrupt conflict
const int TRIG_PIN = 7;
const int ECHO_PIN = 12;

// Quadrature encoders
#define LEFT_A   2
#define LEFT_B   4
#define RIGHT_A  3
#define RIGHT_B  11

// Optional enable pins
const int L_REN = -1;
const int L_LEN = -1;
const int R_REN = -1;
const int R_LEN = -1;

// ---- SPEED LIMITS ----
//tune speed here
const int MAX_SPEED  = 60; //250
const int FWD_SPEED  = 40;  //150;
const int TURN_SPEED = 40;  //130;
const int BACK_SPEED = 30;  //130;
//PWM bias for adjusting right wheel
const int RIGHT_PWM_BIAS = 5;

// -------- RAMP SETTINGS --------
const float ACCEL_PWM_PER_SEC = 125.0;   // reach 50 PWM in 0.4 s
const float DECEL_PWM_PER_SEC = 135.0;   // stop from 50 PWM in 0.28 s

// -------- MOTOR RAMP STATE --------
float currentLeftPWM = 0.0f;
float currentRightPWM = 0.0f;

int targetLeftPWM = 0;
int targetRightPWM = 0;

unsigned long lastRampMs = 0;

// ---- ULTRASONIC SETTINGS ----
const float STOP_DISTANCE_CM = 20.0;
const unsigned long SENSOR_INTERVAL = 100;

// ---- TELEMETRY / SAFETY ----
const unsigned long TELEMETRY_INTERVAL_MS = 100;  // range telemetry
const unsigned long ENC_INTERVAL_MS = 25;         // encoder telemetry at 40 Hz
const unsigned long CMD_WATCHDOG_MS = 250;

// ---- ENCODER STATE ----
volatile long leftCount = 0;
volatile long rightCount = 0;

volatile int lastLeftA = HIGH;
volatile int lastRightA = HIGH;

unsigned long lastTelemetryMs = 0;
unsigned long lastEncMs = 0;
unsigned long lastCmdMs = 0;

// -------- ENCODER ISRs --------
void leftEncoderISR() {
  int a = digitalRead(LEFT_A);
  if (a != lastLeftA) {
    int b = digitalRead(LEFT_B);
    if (b != a) {
      leftCount++;
    } else {
      leftCount--;
    }
    lastLeftA = a;
  }
}

void rightEncoderISR() {
  int a = digitalRead(RIGHT_A);
  if (a != lastRightA) {
    int b = digitalRead(RIGHT_B);
    if (b != a) {
      rightCount++;
    } else {
      rightCount--;
    }
    lastRightA = a;
  }
}

// -------- ULTRASONIC SENSOR FUNCTIONS --------
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 10000);  // reduced timeout

  if (duration == 0) {
    return 999.0;
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

// -------- MOTOR CONTROL HELPERS --------
int clampSpeed(int s) {
  if (s > MAX_SPEED) return MAX_SPEED;
  if (s < -MAX_SPEED) return -MAX_SPEED;
  return s;
}

int applyRightBias(int s) {
  if (s > 0) return clampSpeed(s + RIGHT_PWM_BIAS);
  if (s < 0) return clampSpeed(s - RIGHT_PWM_BIAS);
  return 0;
}

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
//
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  driveBTS7960(L_RPWM, L_LPWM, leftSpeed);
  driveBTS7960(R_RPWM, R_LPWM, applyRightBias(rightSpeed));
}
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
  requestMotorSpeeds(-TURN_SPEED, -TURN_SPEED);
}

void turnRight() {
  requestMotorSpeeds(TURN_SPEED, TURN_SPEED);
}
void quitDrive() {
  applyMotorSpeedsImmediate(0, 0);
}

// -------- SETUP --------
void setup() {
  Serial.begin(115200);

  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LEFT_A, INPUT_PULLUP);
  pinMode(LEFT_B, INPUT_PULLUP);
  pinMode(RIGHT_A, INPUT_PULLUP);
  pinMode(RIGHT_B, INPUT_PULLUP);

  lastLeftA = digitalRead(LEFT_A);
  lastRightA = digitalRead(RIGHT_A);

  attachInterrupt(digitalPinToInterrupt(LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightEncoderISR, CHANGE);

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
  Serial.println("READY");
}

// -------- LOOP --------
void loop() {
  unsigned long now = millis();

  updateMotorRamp(now);

  // 1) Publish ultrasonic telemetry every 100 ms
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    float distance = readDistance();
    Serial.print("RANGE,");
    Serial.println(distance, 2);
  }

  // 2) Publish encoder telemetry
  if (now - lastEncMs >= ENC_INTERVAL_MS) {
    lastEncMs = now;

    long l, r;
    noInterrupts();
    l = leftCount;
    r = rightCount;
    interrupts();

    Serial.print("ENC,L=");
    Serial.print(l);
    Serial.print(",R=");
    Serial.print(r);
    Serial.print(",T=");
    Serial.println(now);
  }

  // 3) Process serial motor commands
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

  // 4) Optional watchdog
  // if ((now - lastCmdMs) > CMD_WATCHDOG_MS) {
  //   stopMotors();
  // }
}
