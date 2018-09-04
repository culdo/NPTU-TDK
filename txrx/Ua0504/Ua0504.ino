char i;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
if (Serial.available()>0) {
  i=Serial.read();
  Serial.println(i);
 // delay(200);
}
}
