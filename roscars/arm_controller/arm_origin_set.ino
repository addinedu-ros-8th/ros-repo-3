#include <Servo.h>

//–– 서보 객체 선언
Servo servo1, servo2, servo3, servo4, fingerServo;

//–– 서보 신호 연결 핀
const int pin1      = 5;   // 1번 joint
const int pin2      = 6;   // 2번 joint
const int pin3      = 9;   // 3번 joint
const int pin4      = 10;  // 4번 joint
const int pinFinger = 11;  // 손가락 서보

//–– origin 각도 정의
const int ORG1 = 140;
const int ORG2 = 40;
const int ORG3 = 130;
const int ORG4 = 100;

//–– 손가락 pick/drop 각도
const int FINGER_PICK = 100;
const int FINGER_DROP = 80;

void setup() {
  Serial.begin(9600);
  //–– 1~4 joint origin 세팅
  servo1.attach(pin1);     servo1.write(ORG1);
  servo2.attach(pin2);     servo2.write(ORG2);
  servo3.attach(pin3);     servo3.write(ORG3);
  servo4.attach(pin4);     servo4.write(ORG4);
  //–– 손가락 pick 위치 초기화
  fingerServo.attach(pinFinger);
  fingerServo.write(FINGER_DROP);

  Serial.println(F("=== Servo Controller ==="));
  Serial.println(F("입력 형식:"));
  Serial.println(F("  1~4 joint: '<id>,<angle>'   (id=1~4)"));
  Serial.println(F("  finger:    'finger,pick' or 'finger,drop'"));
  Serial.println(F("예시: '2,90'  'finger,drop'"));
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  int comma = line.indexOf(',');
  if (comma < 0) {
    Serial.println(F("형식 오류: 'id,angle' 또는 'finger,pick'"));
    return;
  }

  String key = line.substring(0, comma);
  String val = line.substring(comma + 1);
  val.trim();

  // joint 제어
  if (key == "1" || key == "2" || key == "3" || key == "4") {
    int id    = key.toInt();
    int angle = val.toInt();
    if (angle < 0 || angle > 180) {
      Serial.println(F("Error: angle 0~180"));
      return;
    }
    switch(id) {
      case 1: servo1.write(angle); break;
      case 2: servo2.write(angle); break;
      case 3: servo3.write(angle); break;
      case 4: servo4.write(angle); break;
    }
    Serial.print(F("Servo "));
    Serial.print(id);
    Serial.print(F(" → "));
    Serial.print(angle);
    Serial.println(F("°"));
    return;
  }

  // 손가락 제어
  if (key.equalsIgnoreCase("finger")) {
    if (val.equalsIgnoreCase("pick")) {
      fingerServo.write(FINGER_PICK);
      Serial.println(F("Finger → PICK"));
    }
    else if (val.equalsIgnoreCase("drop")) {
      fingerServo.write(FINGER_DROP);
      Serial.println(F("Finger → DROP"));
    }
    else {
      // 숫자로 직접 제어해도 OK
      int angle = val.toInt();
      if (angle >= 0 && angle <= 180) {
        fingerServo.write(angle);
        Serial.print(F("Finger → "));
        Serial.print(angle);
        Serial.println(F("°"));
      } else {
        Serial.println(F("Error: finger angle 0~180 or use pick/drop"));
      }
    }
    return;
  }

  Serial.println(F("Error: id는 1~4 또는 'finger'"));
}
