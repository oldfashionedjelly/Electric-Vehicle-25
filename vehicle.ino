#define PIN_PB_START           0
#define PIN_MTR1_ENCB          3
#define PIN_MTR1_ENCA          2
#define PIN_MTR1_DIR_FWD       7
#define PIN_MTR1_DIR_REV       8
#define PIN_MTR1_PWM           10

#define ENCODER_COUNTS_PER_REV  540     
#define MM_PER_REV              235.6    // 239.3 w/ rubber bands

unsigned long targetDistanceMM = 1000; // Distance in mm (e.g., 1000 mm = 100 cm)
unsigned long targetEncoderCounts;     
unsigned long encoderCount = 0;        
bool isMotorRunning = false;          

void setup() {
  pinMode(PIN_PB_START, INPUT);
  pinMode(PIN_MTR1_ENCA, INPUT);
  pinMode(PIN_MTR1_ENCB, INPUT);

  pinMode(PIN_MTR1_DIR_FWD, OUTPUT);
  pinMode(PIN_MTR1_DIR_REV, OUTPUT);
  pinMode(PIN_MTR1_PWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_MTR1_ENCA), encoderISR, RISING);

  targetEncoderCounts = (targetDistanceMM * ENCODER_COUNTS_PER_REV) / MM_PER_REV;

  Serial.begin(9600);
}

void loop() {
  if (digitalRead(PIN_PB_START) == LOW && !isMotorRunning) {
    encoderCount = 0;  
    isMotorRunning = true;  

    digitalWrite(PIN_MTR1_DIR_FWD, HIGH);
    digitalWrite(PIN_MTR1_DIR_REV, LOW);
    analogWrite(PIN_MTR1_PWM, 255);
  }

  if (isMotorRunning && encoderCount >= targetEncoderCounts) {
    isMotorRunning = false;

    digitalWrite(PIN_MTR1_DIR_FWD, LOW);
    digitalWrite(PIN_MTR1_DIR_REV, LOW);
    analogWrite(PIN_MTR1_PWM, 0);

  }
}

void encoderISR() {
  encoderCount++;
}
