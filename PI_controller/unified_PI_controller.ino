// control_chair_bts7960_ultrasonic_encoder_pi_fixed.ino
// Arduino Mega
//
// Fixes applied vs previous version:
//   1. readDistance() moved OUT of the 50 Hz PI loop into its own 100 ms timer.
//      This prevents pulseIn() from blocking encoder ISRs and corrupting dt.
//   2. dt is now clean — never inflated by pulseIn() blocking time.
//   3. Obstacle check is decoupled from PI timing.
//   4. Serial telemetry format preserved (RANGE,x.xx  /  ENC,L=x,R=x,T=x).
//   5. All other PI logic, watchdog, and motor API unchanged.
//
// Serial command API (115200 baud):
//   forward, backward, turnLeft, turnRight, stop, quit

// =====================================================
// PIN DEFINITIONS
// =====================================================
const int L_RPWM = 5;
const int L_LPWM = 6;
const int R_RPWM = 9;
const int R_LPWM = 10;

const int TRIG_PIN = 7;
const int ECHO_PIN = 8;

#define LEFT_A 3  // INT1 — interrupt-capable on Mega
#define LEFT_B 11
#define RIGHT_A 2  // INT0 — interrupt-capable on Mega
#define RIGHT_B 4

// Set to real pin numbers if R_EN / L_EN are wired to GPIO instead of 5V.
// Leave as -1 if tied directly to 5V.
const int L_REN = -1;
const int L_LEN = -1;
const int R_REN = -1;
const int R_LEN = -1;

// =====================================================
// TIMING CONSTANTS
// =====================================================
const unsigned long CONTROL_INTERVAL_MS = 20;     // 50 Hz PI loop
const unsigned long OBSTACLE_INTERVAL_MS = 100;   // non-blocking ultrasonic poll
const unsigned long ENC_INTERVAL_MS = 25;         // encoder telemetry to Jetson
const unsigned long TELEMETRY_INTERVAL_MS = 100;  // range telemetry to Jetson
const unsigned long CMD_WATCHDOG_MS = 250;        // stop if no command received

// =====================================================
// LIMITS
// =====================================================
const int MAX_PWM = 100;           // Actual is 255, but this is clamped
const int MIN_EFFECTIVE_PWM = 25;  // tune: minimum PWM that actually moves the wheel

// =====================================================
// ULTRASONIC
// =====================================================
const float STOP_DISTANCE_CM = 20.0f;

// =====================================================
// ENCODER STATE (raw, updated by ISRs)
// =====================================================
volatile long leftCount = 0;
volatile long rightCount = 0;
volatile int lastLeftA = HIGH;
volatile int lastRightA = HIGH;

// =====================================================
// TIMING STATE
// =====================================================
unsigned long lastControlMs = 0;
unsigned long lastObstacleMs = 0;  // separate timer for ultrasonic
unsigned long lastEncMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastCmdMs = 0;

// =====================================================
// PI CONTROL STATE
//
// Wheel-speed sign convention (robot frame):
//   +left  speed = left  wheel spinning forward
//   +right speed = right wheel spinning forward
//
// Encoder hardware convention (measured empirically):
//   left  encoder counts UP   when wheel spins forward
//   right encoder counts DOWN when wheel spins forward
//   => negate rightCount delta to get robot-frame speed
//
// Motor hardware convention:
//   left  BTS7960: positive signedPWM -> forward
//   right BTS7960: positive signedPWM -> backward (wiring inverted)
//   => negate rightPWM before calling driveBTS7960()
// =====================================================
float targetLeftTicksPerSec = 0.0f;     // Ramps gradually
float targetRightTicksPerSec = 0.0f;    // Ramps gradually

float desiredLeftTicksPerSec  = 0.0f;   // Changes immediately when a command arrives
float desiredRightTicksPerSec = 0.0f;   // Changes immediately when a command arrives
const float MAX_TARGET_ACCEL_TICKS_PER_SEC2 = 100.0f; // Ramp constant

float measuredLeftTicksPerSec = 0.0f;
float measuredRightTicksPerSec = 0.0f;
float integLeft = 0.0f;
float integRight = 0.0f;
float cmdLeftPWM = 0.0f;
float cmdRightPWM = 0.0f;

long prevLeftCount = 0;
long prevRightCount = 0;

// Cached distance from the non-blocking ultrasonic poll
float cachedDistanceCm = 999.0f;

// =====================================================
// DISCRETE COMMAND TARGETS (ticks/sec)
// Tune these after bench bringup.
// =====================================================
const float FWD_TICKS_PER_SEC = 55.0f;
const float BACK_TICKS_PER_SEC = 55.0f;
const float TURN_TICKS_PER_SEC = 38.0f;

// =====================================================
// PI GAINS
// Start conservative; increase KP first, then KI.
// If wheels oscillate, reduce KI or increase INTEG_LIMIT.
// =====================================================
const float KP = 0.3f;
const float KI = 0.0f;
const float INTEG_LIMIT = 120.0f;

// =====================================================
// MOTION MODE
// =====================================================
enum MotionMode {
  MODE_STOP = 0,
  MODE_FORWARD,
  MODE_BACKWARD,
  MODE_TURN_LEFT,
  MODE_TURN_RIGHT
};
MotionMode currentMode = MODE_STOP;

// =====================================================
// ENCODER ISRs
// =====================================================
void leftEncoderISR() {
  int a = digitalRead(LEFT_A);
  if (a != lastLeftA) {
    int b = digitalRead(LEFT_B);
    leftCount += (b != a) ? 1 : -1;
    lastLeftA = a;
  }
}

void rightEncoderISR() {
  int a = digitalRead(RIGHT_A);
  if (a != lastRightA) {
    int b = digitalRead(RIGHT_B);
    rightCount += (b != a) ? 1 : -1;
    lastRightA = a;
  }
}

// =====================================================
// ULTRASONIC — only called from its own 100 ms timer,
// never from inside the PI loop.
// =====================================================
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 10000);
  if (duration == 0) return 999.0f;
  return (duration / 2.0f) * 0.0343f;
}

// =====================================================
// MOTOR HELPERS
// =====================================================
int clampPWM(int x) {
  if (x > MAX_PWM) return MAX_PWM;
  if (x < -MAX_PWM) return -MAX_PWM;
  return x;
}

void driveBTS7960(int rpwmPin, int lpwmPin, int signedPWM) {
  signedPWM = clampPWM(signedPWM);
  if (signedPWM > 0) {
    analogWrite(rpwmPin, signedPWM);
    analogWrite(lpwmPin, 0);
  } else if (signedPWM < 0) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -signedPWM);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

// +leftNorm  = left  wheel forward
// +rightNorm = right wheel forward
// Right motor wiring is inverted — negate before sending to BTS7960.
void applyWheelPWM(float leftNorm, float rightNorm) {
  driveBTS7960(L_RPWM, L_LPWM, (int)leftNorm);
  driveBTS7960(R_RPWM, R_LPWM, -(int)rightNorm);
}

void hardStopMotors() {
  applyWheelPWM(0, 0);
}

// =====================================================
// MOTION MODE STATE MACHINE
// =====================================================
void resetControllers() {
  integLeft = 0.0f;
  integRight = 0.0f;
  cmdLeftPWM = 0.0f;
  cmdRightPWM = 0.0f;
}

void setMode(MotionMode mode) {
  currentMode = mode;
  resetControllers();

  switch (mode) {
    case MODE_STOP:
      desiredLeftTicksPerSec  = 0.0f;
      desiredRightTicksPerSec = 0.0f;
      break;

    case MODE_FORWARD:
      desiredLeftTicksPerSec  =  FWD_TICKS_PER_SEC;
      desiredRightTicksPerSec =  FWD_TICKS_PER_SEC;
      break;

    case MODE_BACKWARD:
      desiredLeftTicksPerSec  = -BACK_TICKS_PER_SEC;
      desiredRightTicksPerSec = -BACK_TICKS_PER_SEC;
      break;

    case MODE_TURN_LEFT:
      desiredLeftTicksPerSec  = -TURN_TICKS_PER_SEC;
      desiredRightTicksPerSec =  TURN_TICKS_PER_SEC;
      break;

    case MODE_TURN_RIGHT:
      desiredLeftTicksPerSec  =  TURN_TICKS_PER_SEC;
      desiredRightTicksPerSec = -TURN_TICKS_PER_SEC;
      break;
  }

  // Optional hard stop immediately only for STOP *REVIEW*****
  if (mode == MODE_STOP) {
    targetLeftTicksPerSec  = 0.0f;
    targetRightTicksPerSec = 0.0f;
    hardStopMotors();
  }
}

// Gradually ramps target<Left/Right>TicksPerSec
float rampToward(float current, float goal, float maxStep) {
  float delta = goal - current;
  if (delta > maxStep)  return current + maxStep;
  if (delta < -maxStep) return current - maxStep;
  return goal;
}

// =====================================================
// PI HELPERS
// =====================================================
float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float applyMinEffectivePWM(float pwmCmd, float targetTicksPerSec) {
  if (targetTicksPerSec == 0.0f) return 0.0f;
  if (pwmCmd > 0.0f && pwmCmd < MIN_EFFECTIVE_PWM) return (float)MIN_EFFECTIVE_PWM;
  if (pwmCmd < 0.0f && pwmCmd > -MIN_EFFECTIVE_PWM) return -(float)MIN_EFFECTIVE_PWM;
  return pwmCmd;
}

// =====================================================
// PI CONTROL UPDATE — called every CONTROL_INTERVAL_MS.
// dt is always clean: never stretched by pulseIn().
// =====================================================
void updateWheelSpeedControl(float dt) {
  long rawLeft, rawRight;
  noInterrupts();
  rawLeft = leftCount;
  rawRight = rightCount;
  interrupts();

  long dLeft = rawLeft - prevLeftCount;
  long dRight = rawRight - prevRightCount;
  prevLeftCount = rawLeft;
  prevRightCount = rawRight;

  // Robot-frame speeds: negate right delta to match convention
  measuredLeftTicksPerSec = (float)dLeft / dt;
  measuredRightTicksPerSec = -(float)dRight / dt;

  // ---- Left wheel PI ----
  if (targetLeftTicksPerSec == 0.0f) {
    integLeft = 0.0f;
    cmdLeftPWM = 0.0f;
  } else {
    float errLeft = targetLeftTicksPerSec - measuredLeftTicksPerSec;
    integLeft += errLeft * dt;
    integLeft = clampFloat(integLeft, -INTEG_LIMIT, INTEG_LIMIT);
    cmdLeftPWM = KP * errLeft + KI * integLeft;
    cmdLeftPWM = applyMinEffectivePWM(cmdLeftPWM, targetLeftTicksPerSec);
    cmdLeftPWM = clampFloat(cmdLeftPWM, -MAX_PWM, MAX_PWM);
  }

  // ---- Right wheel PI ----
  if (targetRightTicksPerSec == 0.0f) {
    integRight = 0.0f;
    cmdRightPWM = 0.0f;
  } else {
    float errRight = targetRightTicksPerSec - measuredRightTicksPerSec;
    integRight += errRight * dt;
    integRight = clampFloat(integRight, -INTEG_LIMIT, INTEG_LIMIT);
    cmdRightPWM = KP * errRight + KI * integRight;
    cmdRightPWM = applyMinEffectivePWM(cmdRightPWM, targetRightTicksPerSec);
    cmdRightPWM = clampFloat(cmdRightPWM, -MAX_PWM, MAX_PWM);
  }

  applyWheelPWM(cmdLeftPWM, cmdRightPWM);
}

// =====================================================
// SERIAL COMMAND HANDLER
// =====================================================
void processCommand(const String& command) {
  lastCmdMs = millis();

  MotionMode newMode = currentMode;

  if      (command == "forward")   newMode = MODE_FORWARD;
  else if (command == "backward")  newMode = MODE_BACKWARD;
  else if (command == "turnLeft")  newMode = MODE_TURN_LEFT;
  else if (command == "turnRight") newMode = MODE_TURN_RIGHT;
  else if (command == "stop")      newMode = MODE_STOP;
  else if (command == "quit")      newMode = MODE_STOP;

  if (newMode != currentMode) {
    setMode(newMode);
  }
}

// =====================================================
// SETUP
// =====================================================
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

  noInterrupts();
  prevLeftCount = leftCount;
  prevRightCount = rightCount;
  interrupts();

  setMode(MODE_STOP);
  hardStopMotors();

  unsigned long now = millis();
  lastControlMs = now;
  lastObstacleMs = now;
  lastEncMs = now;
  lastTelemetryMs = now;
  lastCmdMs = now;

  Serial.println("READY");
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
  unsigned long now = millis();

  // 1) PI control — 50 Hz, no sensor reads inside here
  if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlMs) / 1000.0f;
  lastControlMs = now;

  float maxStep = MAX_TARGET_ACCEL_TICKS_PER_SEC2 * dt;

  targetLeftTicksPerSec  = rampToward(targetLeftTicksPerSec,
                                      desiredLeftTicksPerSec,
                                      maxStep);

  targetRightTicksPerSec = rampToward(targetRightTicksPerSec,
                                      desiredRightTicksPerSec,
                                      maxStep);
  
  updateWheelSpeedControl(dt);
  }

  // 2) Non-blocking obstacle check — 10 Hz
  //    pulseIn() blocks here for up to 10 ms, which is fine
  //    at 100 ms cadence and does NOT affect PI loop timing.
  if (now - lastObstacleMs >= OBSTACLE_INTERVAL_MS) {
    lastObstacleMs = now;
    cachedDistanceCm = readDistance();
    if (currentMode == MODE_FORWARD && cachedDistanceCm < STOP_DISTANCE_CM) {
      setMode(MODE_STOP);
      Serial.println("OBSTACLE_DETECTED");
    }
  }

  // 3) Range telemetry — 10 Hz, reuses cached value (no extra sensor read)
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    Serial.print("RANGE,");
    Serial.println(cachedDistanceCm, 2);
  }

  // 4) Encoder telemetry — ~40 Hz, unchanged format for Jetson parser
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

  // 5) Serial command receive
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }

  // 6) Watchdog — stop if Jetson goes silent
  if ((now - lastCmdMs) > CMD_WATCHDOG_MS) {
    if (currentMode != MODE_STOP) {
      setMode(MODE_STOP);
    }
  }
}

