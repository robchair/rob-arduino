// Dual encoder bringup test for Arduino Uno
// One interrupt channel per encoder:
//   Left A  -> D2 (interrupt)
//   Left B  -> D4
//   Right A -> D3 (interrupt)
//   Right B -> D5
//
// Serial Monitor must be 115200 baud.

#define LEFT_A   2
#define LEFT_B   4
#define RIGHT_A  3
#define RIGHT_B  5

volatile long leftCount = 0;
volatile long rightCount = 0;

volatile int leftDir = 0;   // +1 or -1
volatile int rightDir = 0;  // +1 or -1

volatile int lastLeftA = HIGH;
volatile int lastRightA = HIGH;

void leftEncoderISR() {
  int a = digitalRead(LEFT_A);
  if (a != lastLeftA) {
    int b = digitalRead(LEFT_B);

    // Direction rule for quadrature when triggering on A
    if (b != a) {
      leftCount++;
      leftDir = +1;
    } else {
      leftCount--;
      leftDir = -1;
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
      rightDir = +1;
    } else {
      rightCount--;
      rightDir = -1;
    }

    lastRightA = a;
  }
}

void setup() {
  Serial.begin(115200);

  // For open-collector encoder outputs, pull-ups are typically appropriate.
  pinMode(LEFT_A, INPUT_PULLUP);
  pinMode(LEFT_B, INPUT_PULLUP);
  pinMode(RIGHT_A, INPUT_PULLUP);
  pinMode(RIGHT_B, INPUT_PULLUP);

  lastLeftA = digitalRead(LEFT_A);
  lastRightA = digitalRead(RIGHT_A);

  attachInterrupt(digitalPinToInterrupt(LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightEncoderISR, CHANGE);

  Serial.println("Dual encoder test started");
  Serial.println("Format: ENC,L=<count>,LD=<dir>,R=<count>,RD=<dir>");
}

void loop() {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  // Print at 20 Hz
  if (now - lastPrint >= 50) {
    lastPrint = now;

    long lCount, rCount;
    int lDir, rDir;

    noInterrupts();
    lCount = leftCount;
    rCount = rightCount;
    lDir = leftDir;
    rDir = rightDir;
    interrupts();

    Serial.print("ENC,L=");
    Serial.print(lCount);
    Serial.print(",LD=");
    if (lDir > 0) Serial.print("CW");
    else if (lDir < 0) Serial.print("CCW");
    else Serial.print("NA");

    Serial.print(",R=");
    Serial.print(rCount);
    Serial.print(",RD=");
    if (rDir > 0) Serial.println("CW");
    else if (rDir < 0) Serial.println("CCW");
    else Serial.println("NA");
  }
}