#include<Servo.h>

Servo servo;
int i = 0;
void setup() {
  servo.attach(10); // D2
}

void loop() {
  for(i=0; i<=50; i+=1) {
    servo.write(i);
    delay(10);
  }
  for(i=50; i>=0; i-=1) {
    servo.write(i);
    delay(10);
  }
  delay(3000);
}
