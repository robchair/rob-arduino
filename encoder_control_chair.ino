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
const int ECHO_PIN = 8;

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
const int MAX_SPEED  = 255;
const int FWD_SPEED  = 150;
const int TURN_SPEED = 130;
const int BACK_SPEED = 130;

// ---- ULTRASONIC SETTINGS ----
const float STOP_DISTANCE_CM = 20.0;
const unsigned long SENSOR_INTERVAL = 100;

// ---- TELEMETRY / SAFETY ----
const unsigned long TELEMETRY_INTERVAL_MS = 100;  // range telemetry
const unsigned long ENC_INTERVAL_MS = 50;         // encoder telemetry at 20 Hz
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

void waitUntilClear() {
  stopMotors();
  Serial.println("OBSTACLE_DETECTED");

  while (true) {
    delay(SENSOR_INTERVAL);
    float distance = readDistance();

    if (distance >= STOP_DISTANCE_CM) {
      Serial.println("PATH_CLEAR");
      break;
    }
  }
}

// -------- MOTOR CONTROL HELPERS --------
int clampSpeed(int s) {
  if (s > MAX_SPEED) return MAX_SPEED;
  if (s < -MAX_SPEED) return -MAX_SPEED;
  return s;
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
  driveBTS7960(R_RPWM, R_LPWM, rightSpeed);
}

void stopMotors() {
  setMotorSpeeds(0, 0);
}

void forward() {
  setMotorSpeeds(FWD_SPEED, -FWD_SPEED);
}

void backward() {
  setMotorSpeeds(-BACK_SPEED, BACK_SPEED);
}

void turnLeft() {
  setMotorSpeeds(0, -TURN_SPEED);
}

void turnRight() {
  setMotorSpeeds(TURN_SPEED, 0);
}

void quitDrive() {
  stopMotors();
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

  // 1) Publish ultrasonic telemetry
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
