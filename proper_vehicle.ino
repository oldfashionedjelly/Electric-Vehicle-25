#define PIN_PB_START           4
#define PIN_MTR1_ENCA          2
#define PIN_MTR1_ENCB          3
#define PIN_MTR1_DIR_FWD       7
#define PIN_MTR1_DIR_REV       8
#define PIN_MTR1_PWM           10

#define ENCODER_COUNTS_PER_REV 240
#define MM_PER_REV             477.522   // 152 mm wheel

const float lengthDistanceMM = 4000.0; // straight distance
const float canDistanceMM    = 250.0;   // lateral offset (0–1000 mm)
const float targetTimeSec    = 10.0;    // total forward + braking time

float Kp = 0.06;
float Ki = 0.0;
float Kd = 0.02;

const float MAX_ACCEL_MMPS2 = 800.0;
const float MAX_DECEL_MMPS2 = 2000.0;
const float MIN_SPEED_MMPS   = 50.0;

volatile long encoderCount = 0;
long targetEncoderCounts = 0;

float cruiseSpeedMMps = 0;
float targetCountsPerSec = 0;
float integral = 0;
float lastError = 0;

int motorPWM = 0;
int lastMotorPWM = 0;
bool isMotorRunning = false;
unsigned long lastControlTime = 0;
unsigned long motorStartTime = 0;

void setup() {
  pinMode(PIN_PB_START, INPUT_PULLUP);
  pinMode(PIN_MTR1_ENCA, INPUT);
  pinMode(PIN_MTR1_ENCB, INPUT);
  pinMode(PIN_MTR1_DIR_FWD, OUTPUT);
  pinMode(PIN_MTR1_DIR_REV, OUTPUT);
  pinMode(PIN_MTR1_PWM, OUTPUT);

  digitalWrite(PIN_MTR1_DIR_FWD, LOW);
  digitalWrite(PIN_MTR1_DIR_REV, LOW);
  analogWrite(PIN_MTR1_PWM, 0);

  attachInterrupt(digitalPinToInterrupt(PIN_MTR1_ENCA), encoderISR, RISING);

  float targetDistanceMM;
  if (canDistanceMM < 1.0) {
    targetDistanceMM = lengthDistanceMM;
  } else {
    float R = (lengthDistanceMM*lengthDistanceMM + 4*canDistanceMM*canDistanceMM) / (8*canDistanceMM);
    float theta = 2.0 * atan(lengthDistanceMM / (2.0 * R));
    targetDistanceMM = R * theta;
  }

  targetEncoderCounts = (long)((targetDistanceMM * ENCODER_COUNTS_PER_REV) / MM_PER_REV);

  cruiseSpeedMMps = targetDistanceMM / targetTimeSec;

  for (int i = 0; i < 20; i++) {
    float t_acc = cruiseSpeedMMps / MAX_ACCEL_MMPS2;
    float t_brake = cruiseSpeedMMps / MAX_DECEL_MMPS2;
    float D_acc = 0.5 * MAX_ACCEL_MMPS2 * t_acc * t_acc;
    float D_brake = 0.5 * MAX_DECEL_MMPS2 * t_brake * t_brake;
    float D_cruise = targetDistanceMM - D_acc - D_brake;
    float t_cruise = targetTimeSec - t_acc - t_brake;
    if (t_cruise <= 0) t_cruise = 0.001;
    cruiseSpeedMMps = D_cruise / t_cruise;
  }

  if (cruiseSpeedMMps < MIN_SPEED_MMPS) cruiseSpeedMMps = MIN_SPEED_MMPS;

  Serial.begin(9600);
}

void loop() {
  if (!isMotorRunning && digitalRead(PIN_PB_START) == LOW) {
    encoderCount = 0;
    lastError = 0;
    integral = 0;
    motorPWM = 0;
    lastMotorPWM = 0;
    lastControlTime = millis();
    motorStartTime = millis();
    isMotorRunning = true;

    digitalWrite(PIN_MTR1_DIR_FWD, HIGH);
    digitalWrite(PIN_MTR1_DIR_REV, LOW);
  }

  if (isMotorRunning) {
    unsigned long now = millis();
    if (now - lastControlTime >= 50) { 
      float dt = (now - lastControlTime) / 1000.0;
      float traveledMM = encoderCount * (MM_PER_REV / ENCODER_COUNTS_PER_REV);
      float remainingMM = targetEncoderCounts * (MM_PER_REV / ENCODER_COUNTS_PER_REV) - traveledMM;

      static long lastEncoderCount = 0;
      float measuredSpeed = (encoderCount - lastEncoderCount) / dt * (MM_PER_REV / ENCODER_COUNTS_PER_REV);
      lastEncoderCount = encoderCount;

      float timeSinceStart = (now - motorStartTime) / 1000.0;
      float rampSpeed = MAX_ACCEL_MMPS2 * timeSinceStart;
      float speedTarget = min(cruiseSpeedMMps, rampSpeed);

      float stoppingDistanceMM = measuredSpeed * measuredSpeed / (2.0 * MAX_DECEL_MMPS2);
      if (remainingMM <= stoppingDistanceMM) {
        speedTarget = sqrt(2.0 * MAX_DECEL_MMPS2 * remainingMM);
      }

      targetCountsPerSec = speedTarget * (ENCODER_COUNTS_PER_REV / MM_PER_REV);
      float error = targetCountsPerSec - measuredSpeed * (ENCODER_COUNTS_PER_REV / MM_PER_REV);
      integral += error * dt;
      float derivative = (error - lastError) / dt;
      float output = Kp * error + Ki * integral + Kd * derivative;

      motorPWM += output;

      motorPWM = constrain(motorPWM, lastMotorPWM - 10, lastMotorPWM + 10); // smooth PWM
      motorPWM = constrain(motorPWM, 0, 255);

      analogWrite(PIN_MTR1_PWM, motorPWM);
      lastMotorPWM = motorPWM;
      lastError = error;
      lastControlTime = now;

      Serial.print("TraveledMM: "); Serial.print(traveledMM);
      Serial.print("  RemainingMM: "); Serial.print(remainingMM);
      Serial.print("  TargetSpeed: "); Serial.print(speedTarget);
      Serial.print("  MeasuredSpeed: "); Serial.print(measuredSpeed);
      Serial.print("  PWM: "); Serial.println(motorPWM);
    }
  }

  if (isMotorRunning && encoderCount >= targetEncoderCounts) {
    isMotorRunning = false;
    analogWrite(PIN_MTR1_PWM, 0);
    digitalWrite(PIN_MTR1_DIR_FWD, LOW);
    digitalWrite(PIN_MTR1_DIR_REV, LOW);
  }
}

void encoderISR() {
  encoderCount++;
}

