#include <Servo.h>

Servo myservo;

void setup()
{
  myservo.attach(9, 300, 2500); // 修正脈衝寬度範圍
  myservo.write(90); // 一開始先置中90度
  delay(3000);
}

void loop()
{
  myservo.write(180); // 一開始先置中90度
  delay(3000);
  myservo.write(90); // 一開始先置中90度
  delay(3000);
}
