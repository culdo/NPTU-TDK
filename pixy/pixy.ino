//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a good place to start if you're just getting started with
// Pixy and Arduino.  This program simply prints the detected object blocks
// (including color codes) through the serial console.  It uses the Arduino's
// ICSP port.  For more information go here:
//
// http://cmucam.org/projects/cmucam5/wiki/Hooking_up_Pixy_to_a_Microcontroller_(like_an_Arduino)
//
// It prints the detected blocks once per second because printing all of the
// blocks for all 50 frames per second would overwhelm the Arduino's serial port.
//
#include <Servo.h>
#include <SPI.h>
#include <Pixy.h>

Servo myservo;
int pos = 0;
// This is the main Pixy object
Pixy pixy;

void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");
  myservo.attach(7);
  pixy.init();
}

void loop()
{
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];

  // grab blocks!
  blocks = pixy.getBlocks();

  // If there are detect blocks, print them!
  if (blocks)
  {
    if (pixy.blocks[j].signature == 1)
    {
      pos ++;
    }
    else if (pixy.blocks[j].signature == 2)
    {
      pos --;
    }
    Serial.println(pixy.blocks[j].signature);
    if (pos >= 180) {
      myservo.write(180);
//      delay(10);
    }
    else if (pos <= 0) {
      myservo.write(0);
//      delay(10);
    }
    else {
      myservo.write(pos);
//      delay(10);
    }
  }


}
