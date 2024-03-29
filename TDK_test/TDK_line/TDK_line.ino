#include <SPI.h>
#define chanel_number 8  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 2  //set PPM signal output pin on the arduino
unsigned long start;
unsigned long now;
bool takeoff = false;
bool land = false;
char openmv;//catch openmv
//////////////////////////////////////////////////////////////////
double ch1;
double ch2;
double ch3;
double ch4;
double ch5;

/*this array holds the servo values for the ppm signal
  change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];
void setup() {
  //initiallize default ppm values
  Serial.begin(9600);
  for (int i = 0; i < chanel_number; i++) {
    ppm[i] = default_servo_value;
  }
  pinMode(12, INPUT);
  pinMode(11, INPUT);
  pinMode(10, INPUT);
  pinMode(9, INPUT);
  pinMode(8, INPUT);
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

  //==========================================================主程式
}
void loop() {
  ch5 = pulseIn(8, HIGH); //飛行模式
  if ( ch5 < 1300) { //遙控模式
    ch1 = pulseIn(12, HIGH); //roll
    ch2 = pulseIn(11, HIGH); //pitch
    ch3 = pulseIn(10, HIGH); //油門
    ch4 = pulseIn(9, HIGH); //YAW
    ch5 = pulseIn(8, HIGH); //mode
    Serial.print("roll:");
    Serial.println(ch1);
    Serial.print("pitch:");
    Serial.println(ch2);
    Serial.print("throttle:");
    Serial.println(ch3);
    Serial.print("yaw:");
    Serial.println(ch4);
    Serial.print("mode:");
    Serial.println(ch5);
    Serial.println();
    ppm[0] = ch1;
    ppm[1] = ch2;
    ppm[2] = ch3;
    ppm[3] = ch4;
    ppm[4] = ch5;
    delay(100);
  }
  else if (ch5 > 1300 && ch5 < 1600) { //任務模式 /
    //    land = false;
    if (takeoff == false) {
      start = millis();
      takeoff = true;
    }
    else {
      now = millis();
      if (now - start < 2000) {
        ppm[0] = 1515;//1500,1525
        ppm[1] = 1457;//1500,1455,1460
        ppm[2] = 1250;
        ppm[3] = 1490;//1500
        ppm[4] = ch5;
      }
      else if (2000 < now - start && now - start < 7000) {
        ppm[0] = 1515;//1500,1525
        ppm[1] = 1457;//1500,1455,1460
        ppm[2] = 1500;
        ppm[3] = 1490;//1500
        ppm[4] = ch5;
      }
      else {
        if (Serial.available() > 0) {
          openmv = Serial.read();
          if (openmv == 'A'){//little left
            ppm[3] = 1450;
          }
          else if (openmv == 'B') {//big left
            ppm[3] = 1400;
          }
           else if (openmv == 'C') {//little right
            ppm[3] = 1550;
          }
           else if (openmv == 'D') {//big right
            ppm[3] = 1600;
          }
           else {//big left
            ppm[3] = 1490;
          }
//          Serial.println(i);
        }
        ppm[0] = 1515;//1500,1525
        ppm[1] = 1457;//1500,1455,1460
        ppm[2] = 1500;
//        ppm[3] = 1490;//1500
        ppm[4] = ch5;
      }
    }
    //    Serial.print("start:");
    //    Serial.println(start);
    //    Serial.print("now:");
    //    Serial.println(now);
    //    Serial.print("油門:");
    //    Serial.println(ppm[2]);
    //    ppm[0] = 1500;
    //    ppm[1] = 1500;
    //    ppm[2] = 1300;
    //    ppm[3] = 1500;
    //    ppm[4] = ch5;
    //    Serial.println(ch5);
    //    Serial.println();
    delay(100);

  }
  //=========================================================================================    
  else { //救機降落
    //    ch1 = pulseIn(12, HIGH); //roll
    //    ch2 = pulseIn(11, HIGH); //pitch
    //    ch3 = pulseIn(10, HIGH); //油門
    //    ch4 = pulseIn(9, HIGH); //YAW
    //    ch5 = pulseIn(8, HIGH); //mode
    if (land == false) {
      start = millis();
      land = true;
    }
    else {
      now = millis();
      if (now - start <= 2000) {
        ppm[0] = 1500;
        ppm[1] = 1500;
        ppm[2] = 1470;//1450
        ppm[3] = 1500;
        ppm[4] = 1800;
      }
      else if ( now - start <= 3000 ) {
        ppm[0] = 1500;
        ppm[1] = 1500;
        ppm[2] = 1440;//1250
        ppm[3] = 1500;
        ppm[4] = 1800;
      }
      else {
        ppm[0] = 1470;
        ppm[1] = 1900;
        ppm[2] = 1000;
        ppm[3] = 1000;
        ppm[4] = 1800;
      }
    }
    //    ppm[0] = 1470;
    //    ppm[1] = 1470;
    //    ppm[2] = 1000;
    //    ppm[3] = 1470;
    //    ppm[4] = 1800;
    //    delay(100);
    //    Serial.print("start:");
    //    Serial.println(start);
    //    Serial.print("now:");
    //    Serial.println(now);
    //    Serial.print("油門:");
    //    Serial.println(ppm[2]);
    //    Serial.println(ppm[3]);
    delay(100);
  }

}
//====================================================PPM產生
ISR(TIMER1_COMPA_vect) {
  static boolean state = true;

  TCNT1 = 0;

  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else {        //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    digitalWrite(sigPin, !onState);
    state = true;

    if (cur_chan_numb >= chanel_number) {
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;//
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }
  }
}
