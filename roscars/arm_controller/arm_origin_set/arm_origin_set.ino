#include <Servo.h>

#define ORG0 110
#define ORG1  25
#define ORG2 110
#define ORG3 100
#define ORG4  65
#define FINGER_PICK 85
#define FINGER_DROP 40

//–– 디지털 핀 번호 정의
const int servoPin0      = 3;
const int servoPin1      = 5;
const int servoPin2      = 6;
const int servoPin3      = 9;
const int servoPin4      = 10;  // 아날로그 0번(핀 14)
const int servoPinFinger = 11;  // 아날로그 1번(핀 15)

Servo servo0, servo1, servo2, servo3, servo4, servoFinger;

void flushSerial() {
  while (Serial.available()) Serial.read();
}

void setup() {
  Serial.begin(9600);

  //–– 서보 모터 attach
  servo0.attach(servoPin0);
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servoFinger.attach(servoPinFinger);

  //–– 초기 위치 세팅
  servo0.write(ORG0);
  servo1.write(ORG1);
  servo2.write(ORG2);
  servo3.write(ORG3);
  servo4.write(ORG4);
  servoFinger.write(FINGER_DROP);

  Serial.println("=== Servo Controller (direct) ===");
  Serial.println("입력 형식:");
  Serial.println("  0~4 joint: '<id>,<angle>'   (id=0~4)");
  Serial.println("  finger:    'finger,pick' or 'finger,drop'");
  flushSerial();
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.replace("\r", ""); 
  line.trim();
  if (line.length() == 0) return;

  int idx = 0;
  while (idx < line.length()) {
    int sp = line.indexOf(' ', idx);
    if (sp == -1) sp = line.length();
    String token = line.substring(idx, sp);
    token.trim();

    int comma = token.indexOf(',');
    if (comma > 0) {
      String key = token.substring(0, comma);
      String val = token.substring(comma + 1);
      val.trim();

      int angle = val.toInt();
      if (key == "0")      { servo0.write(angle);      Serial.println("Servo 0 → " + String(angle) + "°"); }
      else if (key == "1") { servo1.write(angle);      Serial.println("Servo 1 → " + String(angle) + "°"); }
      else if (key == "2") { servo2.write(angle);      Serial.println("Servo 2 → " + String(angle) + "°"); }
      else if (key == "3") { servo3.write(angle);      Serial.println("Servo 3 → " + String(angle) + "°"); }
      else if (key == "4") { servo4.write(angle);      Serial.println("Servo 4 → " + String(angle) + "°"); }
      else if (key.equalsIgnoreCase("finger")) {
        if (val.equalsIgnoreCase("pick")) {
          servoFinger.write(FINGER_PICK);
          Serial.println("Finger → PICK");
        } else if (val.equalsIgnoreCase("drop")) {
          servoFinger.write(FINGER_DROP);
          Serial.println("Finger → DROP");
        } else {
          servoFinger.write(angle);
          Serial.println("Finger → " + String(angle) + "°");
        }
      } else {
        Serial.println("Unknown key: " + key);
      }
    }
    idx = sp + 1;
  }
}
