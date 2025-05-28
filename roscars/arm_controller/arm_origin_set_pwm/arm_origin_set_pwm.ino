#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_MIN 150
#define SERVO_MAX 600

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// –– 채널 번호 정의 (pin → channel로 대응)
const int ch1      = 0;  // 1번 joint
const int ch2      = 1;  // 2번 joint
const int ch3      = 2;  // 3번 joint
const int ch4      = 3;  // 4번 joint
const int ch5      = 4;  // 5번 joint
const int chFinger = 5;  // 손가락

// const int ch1      = 10;  // 1번 joint
// const int ch2      = 11;  // 2번 joint
// const int ch3      = 12;  // 3번 joint
// const int ch4      = 13;  // 4번 joint
// const int ch5      = 14;  // 5번 joint
// const int chFinger = 15;  // 손가락

//–– origin 각도 정의
const int ORG1 = 110;
const int ORG2 = 25;
const int ORG3 = 110;
const int ORG4 = 100;
const int ORG5 = 65;

//–– 손가락 pick/drop 각도
const int FINGER_PICK = 85;
const int FINGER_DROP = 40;

int angleToPWM(float angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  return SERVO_MIN + (angle / 180.0) 
  * (SERVO_MAX - SERVO_MIN);
}

void flushSerial() {
  while (Serial.available()) {
    Serial.read();
  }
}


void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  //–– 초기 위치
  pwm.setPWM(ch1, 0, angleToPWM(ORG1));
  pwm.setPWM(ch2, 0, angleToPWM(ORG2));
  pwm.setPWM(ch3, 0, angleToPWM(ORG3));
  pwm.setPWM(ch4, 0, angleToPWM(ORG4));
  pwm.setPWM(ch5, 0, angleToPWM(ORG5));
  pwm.setPWM(chFinger, 0, angleToPWM(FINGER_DROP));

  Serial.println(F("=== Servo Controller (PCA9685) ==="));
  Serial.println(F("입력 형식:"));
  Serial.println(F("  1~5 joint: '<id>,<angle>'   (id=1~5)"));
  Serial.println(F("  finger:    'finger,pick' or 'finger,drop'"));

  flushSerial();
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.replace("\r", ""); 
  line.trim();
  if (line.length() == 0) return;

  int start = 0;
  while (start < line.length()) {
    int end = line.indexOf(' ', start);
    if (end == -1) end = line.length();

    String token = line.substring(start, end);
    token.trim();

    int comma = token.indexOf(',');
    if (comma > 0) {
      String key = token.substring(0, comma);
      String val = token.substring(comma + 1);
      val.trim();
      int angle = val.toInt();

      if (angle < 0 || angle > 180) {
        Serial.print(F("Invalid angle: "));
        Serial.println(val);
      } else if (key == "1" || key == "2" || key == "3" || key == "4" || key == "5") {
        int id = key.toInt();
        int channel = -1;
        switch (id) {
          case 1: channel = ch1; break;
          case 2: channel = ch2; break;
          case 3: channel = ch3; break;
          case 4: channel = ch4; break;
          case 5: channel = ch5; break;
        }
        if (channel >= 0) {
          pwm.setPWM(channel, 0, angleToPWM(angle));
          Serial.print(F("Servo "));
          Serial.print(id);
          Serial.print(F(" → "));
          Serial.print(angle);
          Serial.println(F("°"));
        }
      } else if (key.equalsIgnoreCase("finger")) {
        if (val.equalsIgnoreCase("pick")) {
          pwm.setPWM(chFinger, 0, angleToPWM(FINGER_PICK));
          Serial.println(F("Finger → PICK"));
        } else if (val.equalsIgnoreCase("drop")) {
          pwm.setPWM(chFinger, 0, angleToPWM(FINGER_DROP));
          Serial.println(F("Finger → DROP"));
        } else {
          int fingerAngle = val.toInt();
          if (fingerAngle >= 0 && fingerAngle <= 180) {
            pwm.setPWM(chFinger, 0, angleToPWM(fingerAngle));
            Serial.print(F("Finger → "));
            Serial.print(fingerAngle);
            Serial.println(F("°"));
          } else {
            Serial.print(F("Invalid finger angle: "));
            Serial.println(val);
          }
        }
      } else {
        Serial.print(F("Unknown key: "));
        Serial.println(key);
      }
    }

    start = end + 1;
  }
}
