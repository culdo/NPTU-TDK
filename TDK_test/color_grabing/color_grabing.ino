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

#include <SPI.h>
#include <Pixy.h>
#define colors 2

// This is the main Pixy object
Pixy pixy;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");

  pixy.init();
}

void loop()
{
  get_color_info();
    //  static int i = 0;
    //  int j;
    //  uint16_t blocks;
    //  char buf[32];
    //
    //  int center_x[colors] = {0};
    //  int center_y[colors] = {0};
    //  int our_blocks[colors] = {0};
    //  int c_idx;
    //
    //  // grab blocks!
    //  blocks = pixy.getBlocks();
    //
    //  // If there are detect blocks, print them!
    //  if (blocks)
    //  {
    //    i++;
    //
    //    // do this (print) every 50 frames because printing every
    //    // frame would bog down the Arduino
    //    if (i % 50 == 0)
    //    {
    //      sprintf(buf, "Detected %d:\n", blocks);
    //      Serial.print(buf);
    //      for (j = 0; j < blocks; j++)
    //      {
    //        sprintf(buf, "  block %d: ", j);
    //        Serial.print(buf);
    //        pixy.blocks[j].print();
    //        // 1是紅綠燈紅色,
    //        if(pixy.blocks[j].signature==1 || pixy.blocks[j].signature==2) {
    //          c_idx = pixy.blocks[j].signature-1;
    //          our_blocks[c_idx] += 1;
    //          center_x[c_idx] += pixy.blocks[j].x;
    //          center_y[c_idx] += pixy.blocks[j].y;
    //        }
    //      }
    //      for(int k=0; k<colors; k++) {
    //        if(our_blocks[k]) {
    //          center_x[k] = int(center_x[k] / our_blocks[k]);
    //          center_y[k] = int(center_y[k] / our_blocks[k]);
    //          Serial.println(String("")+"Color "+int(k+1)+" Center:");
    //          Serial.println(String("")+"x: "+center_x[k]+" y: "+center_y[k]);
    //  //        return our_blocks[0];
    //        }
    //      }
    //    }
    //  }
}

int get_color_info(void )
{
  static int frames = 0;
  int j;
  uint16_t blocks;
  char buf[32];
  int center_x[colors] = {0};
  int center_y[colors] = {0};
  int our_blocks[colors] = {0};
  int c_idx;

  // grab blocks!
  blocks = pixy.getBlocks();

  // If there are detect blocks, print them!
  if (blocks)
  {
    frames++;

    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (frames % 50 == 0)
    {
            sprintf(buf, "Detected %d:\n", blocks);
      //      BT.print(buf);
            Serial.print(buf);
      for (j = 0; j < blocks; j++)
      {
                sprintf(buf, "  block %d: ", j);
        //        BT.print(buf);
                Serial.print(buf);
                pixy.blocks[j].print();
        // 1是紅綠燈紅色,
        if (pixy.blocks[j].signature == 1 || pixy.blocks[j].signature == 2) {
          c_idx = pixy.blocks[j].signature - 1;
          our_blocks[c_idx] += 1;
          center_x[c_idx] += pixy.blocks[j].x;
          center_y[c_idx] += pixy.blocks[j].y;
        }
      }
      for (int k = 0; k < colors; k++) {
        if (our_blocks[k]) {
          center_x[k] = int(center_x[k] / our_blocks[k]);
          center_y[k] = int(center_y[k] / our_blocks[k]);
//          BT.println(String("") + "Color " + int(k + 1) + " Center:");
          Serial.println(String("") + "Color " + int(k + 1) + " Center:");
//          BT.println(String("") + "x: " + center_x[k] + " y: " + center_y[k]);
          Serial.println(String("") + "x: " + center_x[k] + " y: " + center_y[k]);
        }
      }
      return our_blocks[0];
    }
  }
}

