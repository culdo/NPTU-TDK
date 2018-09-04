int px4Pin = 7;   // LED connected to digital pin 13
int inPin = 3;     // pushbutton connected to digital pin 7
int val = 0;       // variable to store the read value

void setup()
{
  pinMode(px4Pin, OUTPUT);      // sets the digital pin 13 as output
  pinMode(inPin, INPUT);        // sets the digital pin 7 as input
}

void loop()
{
  val = digitalRead(inPin);     // read the input pin
//  digitalWrite(px4Pin,  digitalRead(inPin));// sets the LED to the button's value
  if(val==LOW) {
    PORTD &= B01111111;
    delayMicroseconds(0.01);
  }else if(val==HIGH) {
    PORTD |= B10000000;
    delayMicroseconds(0.01);
  }
}
