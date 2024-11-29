#define PIN_PB_START           0
#define PIN_MTR1_ENCB          3
#define PIN_MTR1_ENCA          2
#define PIN_MTR1_DIR_FWD       7
#define PIN_MTR1_DIR_REV       8
#define PIN_MTR1_PWM           10

#define ENCODER_COUNTS_PER_REV  540     
#define MM_PER_REV              235.62     
#define ENCODER_COUNTS_90_DEG   135   

unsigned long motorStartTime = 0;  
bool isMotorRunning = false;    

void setup() {
  pinMode(PIN_PB_START, INPUT);

  pinMode(PIN_MTR1_DIR_FWD, OUTPUT);
  pinMode(PIN_MTR1_DIR_REV, OUTPUT);
  pinMode(PIN_MTR1_PWM, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  if (digitalRead(PIN_PB_START) == LOW && !isMotorRunning) {
    motorStartTime = millis();  
    isMotorRunning = true;  

    digitalWrite(PIN_MTR1_DIR_FWD, HIGH);
    digitalWrite(PIN_MTR1_DIR_REV, LOW);
    analogWrite(PIN_MTR1_PWM, 255);
  }

  if (isMotorRunning && (millis() - motorStartTime >= 5000)) {
    isMotorRunning = false;

    digitalWrite(PIN_MTR1_DIR_FWD, LOW);
    digitalWrite(PIN_MTR1_DIR_REV, LOW);
    analogWrite(PIN_MTR1_PWM, 0);
  }

  delay(50); 
}
