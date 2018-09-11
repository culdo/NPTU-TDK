#include <NewPing.h>
#include <SPI.h>
#define chanel_number 8  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 2  //set PPM signal output pin on the arduino
#define TRIG_PIN 7
#define ECHO_PIN 6
#define MAX_DISTANCE 200
#define roll_center 1460  //1465,1478
#define pitch_center 1445  //1435前進用1442微微後退
#define yaw_center 1484
//#define debug false

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

unsigned long start, now, timer;
bool is_takeoff = false;
bool is_land = false;
bool is_sonic_fly = false;
//bool switched = false;

//////////////////////////////////////////////////////////////////
int ch5;
/*this array holds the servo values for the ppm signal
  change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];
int ppm_value;
int updated_times = 0;
int sonar_cm;
//openmv
char cmd[10];
int openmv;

void setup() {
  //initiallize default ppm values
  //  if(debug == true) {
  Serial.begin(9600);
  //  }
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

  timer = millis();
  //==========================================================主程式
}

void loop() {
  //  int down_cm;
  //  Serial.println(sonar.ping_cm());
  debug();
  ch5 = pulseIn(8, HIGH); //飛行模式
  ppm[4] = ch5; //mode
  if ( ch5 < 1300) { //遙控模式
    rc_mode();
  }
  else if (ch5 >= 1300 && ch5 < 1600) { //任務模式
    mission_mode();
    //    debug();
  }
  else { //救機降落
    land_mode();
  }

  //  if(ppm[2]>=1500) { //緊急停止
  //    is_takeoff_mode()
  //  }
  delay(100);
}
//void switch_mode(void ) {
//  switch(mode) {
//    case 1;
//    case 2;
//    cae
//  }
//}

void rc_mode(void ) {
  is_takeoff = false;
  is_land = false;
  is_sonic_fly = false;

  ppm[0] = pulseIn(12, HIGH); //roll
  ppm[1] = pulseIn(11, HIGH); //pitch
  ppm[2] = pulseIn(10, HIGH); //油門
  ppm[3] = pulseIn(9, HIGH); //YAW
  ppm[4] = pulseIn(8, HIGH); //mode
}

void mission_mode(void ) {
  is_land = false;
  if (is_takeoff == false) {
    start = millis();
    is_takeoff = true;
    ppm_value = 1435;
  }
  else {
    now = millis();
    if (now - start <= 15000) {
      ppm[0] = roll_center;//1500,1460
      ppm[1] = pitch_center;//1435,1450
      ppm[2] = 1427; //1465
      ppm[3] = yaw_center;//1500
    }
    else {
      debug();
      sonar_cm = sonar.ping_cm();
      ppm[2] = ppm_value; //1470
      now = millis();
      if (is_sonic_fly == false) {
        if ((sonar_cm < 75) && ((now - timer) >= 3000)) {
          timer = millis();
          ppm_value += 5;
        } else if (sonar_cm >= 75) {
          is_sonic_fly = true;
        }
      }
      else {
        //       openmv
        if (Serial.available() > 0) {
        if (Serial.read() == '\n') {
            for(int i=0; i<4; i++) {
              cmd[i] = Serial.read();
            }
        }
        }
        ppm[3] = atoi(cmd);
      }
    }
  }
}

void land_mode(void ) {
  is_sonic_fly = false;
  is_takeoff = false;
  if (is_land == false) {
    start = millis();
    is_land = true;
  } else {
    now = millis();
    if (now - start <= 2000) {
      ppm[0] = roll_center;
      ppm[1] = pitch_center;
      ppm[2] = 1460;//1450
      ppm[3] = yaw_center;
      ppm[4] = 1800;
    }
    else if ( now - start <= 3000 ) {
      ppm[0] = roll_center;
      ppm[1] = pitch_center;
      ppm[2] = 1455;//1250
      ppm[3] = yaw_center;
      //        ppm[4] = 1800;
    }
    else if (now - start <= 4000 ) {
      ppm[0] = roll_center;
      ppm[1] = pitch_center;
      ppm[2] = 1450;//1250
      ppm[3] = yaw_center;
      //        ppm[4] = 1800;
    }
    else {
      ppm[0] = 1470;
      ppm[1] = 1900;
      ppm[2] = 1000;
      ppm[3] = 1000;
      ppm[4] = 1800;
    }
  }
}

void debug(void) {
  if ( ch5 < 1300) {
    Serial.println("================Radio Mode================");
    Serial.print("roll:");
    Serial.println(ppm[0]);
    Serial.print("pitch:");
    Serial.println(ppm[1]);
    Serial.print("throttle:");
    Serial.println(ppm[2]);
    Serial.print("yaw:");
    Serial.println(ppm[3]);
    Serial.print("mode:");
    Serial.println(ppm[4]);
    Serial.print("start:");
    Serial.println(start / 1000);
    Serial.print("now:");
    Serial.println(now / 1000);
  } else if (ch5 >= 1300 && ch5 < 1600) {
    Serial.println("================Mission Mode================");
    Serial.print("throttle:");
    Serial.println(ppm[2]);
    Serial.print("yaw:");
    Serial.println(ppm[3]);
    Serial.print("start:");
    Serial.println(start / 1000);
    Serial.print("timer:");
    Serial.println(timer / 1000);
    Serial.print("now:");
    Serial.println(now / 1000);
    Serial.print("cm:");
    Serial.println(sonar_cm);
  } else {
    Serial.println("================Save-Land Mode================");
    Serial.print("throttle:");
    Serial.println(ppm[2]);
    Serial.print("start:");
    Serial.println(start / 1000);
    Serial.print("now:");
    Serial.println(now / 1000);

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

