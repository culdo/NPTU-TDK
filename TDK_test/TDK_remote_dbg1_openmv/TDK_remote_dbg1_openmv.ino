#include <NewPing.h>
#include <SPI.h>
#define chanel_number 8          //set the number of chanels
#define default_servo_value 1000 //set the default servo value
#define PPM_FrLen 22500          //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300         //set the pulse length
#define onState 1                //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 2                 //set PPM signal output pin on the arduino
#define TRIG_PIN 13
#define ECHO_PIN 12
#define ch1_pin 3
#define ch2_pin 4
#define ch3_pin 5
#define ch4_pin 6
#define ch7_pin 7
#define MAX_DISTANCE 200
#define roll_center 1460  //好螺旋1462
#define pitch_center 1415 //好螺旋1419,1422
#define yaw_center 1494//1485
//#include <SoftwareSerial.h>   // 引用程式庫
#include <Pixy.h>
#define colors 2

//debug選項: false=關閉debug 'P'=電腦PC 'B'=藍芽Bluetooth
#define debug 'B'

// 定義連接藍牙模組的序列埠
//SoftwareSerial 'B'(3, 12); // 接收腳, 傳送腳
char val; // 儲存接收資料的變數

//#define debug false

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
// This is the main Pixy object
Pixy pixy;

unsigned long now;
unsigned long start, timer, before, before_op = 0;
unsigned long interval = 0;
bool is_takeoff = false;
bool is_land = false;
bool is_sonic_fly = false;
bool is_get_red = false;
bool is_error = false;
bool is_high = false;
bool open_pid = false;
//bool switched = false;

//////////////////////////////////////////////////////////////////
//int ppm[4];
/*this array holds the servo values for the ppm signal
  change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];
int ppm_value;
int updated_times = 0;
int sonar_cm = 0;
//openmv
char cmd[4];
char garbage[4];
int openmv;
int c = 0;
float control;
float n_speed = 1482;//1484
int new_speed;
float pre_e = 0;
int set_d = 75;
float error;
float s = 0;
float kp = 0.13;//0.1
float ki = 0;
float kd = 0.42;//0.25
//float kp1= 0.15;//0.1
//float ki1 = 0;
//float kd1 = 0.2;//0.3
//int error_ppm=2555;

void setup()
{
  //initiallize default ppm values
  // 設為115200平滑接收監控訊息
  Serial2.begin(115200);
#if debug == 'B'
  Serial1.begin(115200);
#elif debug == 'P'
  //藍芽鮑率
  Serial.begin(115200);
#endif

  for (int i = 0; i < chanel_number; i++)
  {
    ppm[i] = default_servo_value;
  }
  pinMode(ch1_pin, INPUT);
  pinMode(ch2_pin, INPUT);
  pinMode(ch3_pin, INPUT);
  pinMode(ch4_pin, INPUT);
  pinMode(ch7_pin, INPUT);
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState); //set the PPM signal pin to the default state (off)
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;             // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);   // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

  // timer = millis();
  // before = millis();
  now = millis();

  pixy.init();
}

//==========================================================主程式
void loop()
{

  //每隔0.4秒藍芽發送紀錄
#if debug != false
  if ((millis() - interval) > 400)
  {
    print_status();
    //      get_color_info();
    interval = millis();
  }
#endif

  if (ppm[2] >= 1520 || is_error == true)
  {
    is_error = true;
    land_mode();
  }
  else
  {
    ppm[4] = pulseIn(ch7_pin, HIGH); //飛行模式
    //    ppm[4] = ppm[4];                 //mode

    //    if (ppm[4] < 1300 && (!is_sonic_fly))
    if (ppm[4] < 1300) //遙控模式
    {
      if (!is_sonic_fly)
        rc_mode();
      //      else
      //        error_ppm = ppm[4];
    }
    else if (ppm[4] >= 1300 && ppm[4] < 1600)
    { //任務模式
      mission_mode();
    }
    else
    { //降落模式
      land_mode();
    }
  }

  //  註解下行允許接收openmv
  //    delay(100);
}

void rc_mode(void)
{
  is_takeoff = false;
  is_land = false;
  is_sonic_fly = false;
  is_get_red = false;
  open_pid = false;
  //  is_error = false;
  int yaw;
  // 解鎖
  if (millis() - start > 1000)
  {
    if ((pulseIn(ch4_pin, HIGH)) > 1900)
    {
      start = millis();
      ppm[3] = 1910;
    }
    else
    {
      ppm[3] = pulseIn(ch4_pin, HIGH); //yaw
    }
  }
  // 為了Pixy讓出三隻plueIn腳(Uno板)
  ppm[0] = pulseIn(ch1_pin, HIGH);       //roll
  ppm[1] = pulseIn(ch2_pin, HIGH);       //pitch
  ppm[2] = pulseIn(ch3_pin, HIGH) - 150; //油門
  //  ppm[4] = pulseIn(ch7_pin, HIGH);       //mode
}

void mission_mode(void)
{
  unsigned long buf;
  is_land = false;
  if (is_takeoff == false)
  {
    start = millis();
    is_takeoff = true;
    // ppm_value = 1455;
    ppm_value = 1478;
    ppm[0] = roll_center;  //1500,1460
    ppm[1] = pitch_center; //1435,1450
    ppm[3] = yaw_center;
  }
  else
  {
        now = millis();
    //    if ((now - start) <= 5000)//5000
    //    {
    //      ppm[0] = roll_center;  //1500,1460
    //      ppm[1] = pitch_center; //1435,1450
    //      ppm[2] = 1420;         //1465
    //      ppm[3] = yaw_center;   //1500
    //    }
    //    else
    //    {
    is_sonic_fly = true;

    if ((millis() - before) >= 1000)
    {
      buf = sonar.ping_cm();
      if (buf > 15)
      {
        sonar_cm = buf;
      }
      before = millis();
    }
    //        if ((sonar_cm < 75) && ((millis() - timer) >= 1000) && (sonar_cm > 15))
    //        {
    //          timer = millis();
    //          ppm_value += 2;
    //        }
    if (sonar_cm >= 50 && open_pid == false) {
      open_pid = true;
    }
    if (open_pid == true) {
      alt_pid();
      //        //        openmv
      //        if (millis() - before_op <= 1000) {
      //openmv-pid============================
//      if (Serial2.available() > 0) {
//        if (Serial2.read() == '\n') {
//          Serial2.readBytes(cmd, 4);
////          Serial.println(cmd);
//        }
//        if (atoi(cmd) > 1400 && atoi(cmd) < 1600 )
//          ppm[3] = atoi(cmd);
//      }
      //============================
      //        }
      //        else if (millis() - before_op <= 2000) {
      //          ppm[3] = yaw_center;
      //          ppm[1] = pitch_center - 10;
      //        }
      //        else {
      //          before_op = millis();
      //        }
    }
    ppm[2] = ppm_value; //1440

    //      }
    // if (sonar_cm > 75 || is_high == true)
    // {
    //   is_high = true;

    //   if (is_sonic_fly == false)
    //   {
    //     if ((sonar_cm > 75) && ((millis() - timer) >= 1000))
    //     {
    //       timer = millis();
    //       ppm_value -= 2;
    //     }
    //     else if (sonar_cm <= 75)
    //     {
    //       is_sonic_fly = true;
    //     }
    //   }
    // if ((sonar_cm < 75) && ((millis() - timer) >= 1000) && (sonar_cm > 15))
    // {
    //   timer = millis();
    //   ppm_value += 2;
    // }
    // else if (sonar_cm >= 75)
    // {
    //   is_sonic_fly = true;
    // }
    // }
    //      else
    //      {
    //        if (!is_get_red)
    //        {
    //          is_get_red = get_color_info();
    //          //往前ppm = 中心ppm - 差值
    //          ppm[1] = pitch_center - 10;
    //        }
    //        else
    //        {
    //          ppm[1] = pitch_center;
    //        }
    //      }


  }

  //      else {
  //        //        openmv
  //        if (millis() - before <= 500) {
  //          if (Serial1.available() > 0) {
  //            if (Serial1.read() == '\n') {
  //              Serial1.readBytes(cmd, 4);
  //            }
  //            //        }
  //            //        Serial1.readBytes(garbage, 10);
  //            ppm[1] = pitch_center;
  //            ppm[3] = atoi(cmd);
  //          }
  //        }
  //        else if (millis() - before <= 1500) {
  //          ppm[3] = yaw_center;
  //          ppm[1] = pitch_center - 10;
  //        }
  //        else {
  //          before = millis();
  //        }
  //      }

}

void land_mode(void)
{
  is_sonic_fly = false;
  //  is_takeoff = false;
  is_get_red = false;
  open_pid = false;
  if (is_land == false)
  {
    before = 0;
    ppm[0] = roll_center;
    ppm[1] = pitch_center;
    ppm[2] = ppm_value; //1480;//1450
    ppm[3] = yaw_center;
    start = millis();
    is_land = true;
  }
  else
  {
    now = millis();
    if (ppm[2] > 1430)
    {
      if (now - before >= 2750)
      {
        before = millis();
        ppm[2] -= 5;
      }
    }
    else
    {
      ppm[0] = 1470;
      ppm[1] = 1900;
      ppm[2] = 1000;
      ppm[3] = 1000;
      ppm[4] = 1800;
    }

    if (ppm[2] <= 1450)
    {
      is_sonic_fly = false;
    }
    //    if (now - start <= 2000) {
    //      ppm[0] = roll_center;
    //      ppm[1] = pitch_center;
    //      ppm[2] = 1460;//1450
    //      ppm[3] = yaw_center;
    //      ppm[4] = 1800;
    //    }
    //    else if ( now - start <= 3000 ) {
    //      ppm[0] = roll_center;
    //      ppm[1] = pitch_center;
    //      ppm[2] = 1455;//1250
    //      ppm[3] = yaw_center;
    //      //        ppm[4] = 1800;
    //    }
    //    else if (now - start <= 4000 ) {
    //      ppm[0] = roll_center;
    //      ppm[1] = pitch_center;
    //      ppm[2] = 1450;//1250
    //      ppm[3] = yaw_center;
    //      //        ppm[4] = 1800;
    //    }
    //    else {
    //      ppm[0] = 1470;
    //      ppm[1] = 1900;
    //      ppm[2] = 1000;
    //      ppm[3] = 1000;
    //      ppm[4] = 1800;
    //    }
    //  }
  }
}
void print_status()
{
  // ==========================只能選擇一種debug方式其他Serial請comment掉!!!==========================
  char cgy[10];
#if debug == 'B'
  //    Serial1.print("ERROR_MODE_PPM: ");
  //    Serial1.println(error_ppm);
  if (ppm[4] < 1300)
  {
    Serial1.println("================Radio Mode================");
  }
  else if (ppm[4] >= 1300 && ppm[4] < 1600)
  {
    Serial1.println("================Mission Mode================");
    Serial1.print("now-start: ");
    Serial1.println((now - start) / 1000);
    Serial1.print("cm: ");
    Serial1.println(sonar_cm);
  }
  else
  {
    Serial1.println("================Save-Land Mode================");
    Serial1.print("now-start: ");
    Serial1.println((now - start) / 1000);
    Serial1.print("before: ");
    Serial1.println(before / 1000);
  }

  Serial1.print("mode: ");
  Serial1.println(ppm[4]);
  Serial1.print("throttle: ");
  Serial1.println(ppm[2]);
  Serial1.print("roll: ");
  if (ppm[0] == roll_center)
  {
    Serial1.println("Center");
  }
  else
  {
    sprintf(cgy, "%+5d", (ppm[0] - roll_center));
    Serial1.println(cgy);
  }
  Serial1.print("pitch: ");
  if (ppm[1] == pitch_center)
  {
    Serial1.println("Center");
  }
  else
  {
    sprintf(cgy, "%+5d", (ppm[1] - pitch_center));
    Serial1.println(cgy);
  }
  Serial1.print("yaw: ");
  if (ppm[3] == yaw_center)
  {
    Serial1.println("Center");
  }
  else
  {
    sprintf(cgy, "%+5d", (ppm[3] - yaw_center));
    Serial1.println(cgy);
  }
#elif debug == 'P'

  if (ppm[4] < 1300)
  {
    Serial.println("================Radio Mode================");
  }
  else if (ppm[4] >= 1300 && ppm[4] < 1600)
  {
    Serial.println("================Mission Mode================");
    Serial.print("now-start: ");
    Serial.println((now - start) / 1000);
    Serial.print("cm: ");
    Serial.println(sonar_cm);
    //          Serial.println(now);
    //          Serial.println(before_op);
    //      Serial.println(start);
  }
  else
  {
    Serial.println("================Save-Land Mode================");
    Serial.print("now-start: ");
    Serial.println((now - start) / 1000);
    Serial.print("before: ");
    Serial.println(before / 1000);
  }

  Serial.print("mode: ");
  Serial.println(ppm[4]);
  Serial.print("throttle: ");
  Serial.println(ppm[2]);
  Serial.print("roll: ");
  if (ppm[0] == roll_center)
  {
    Serial.println("Center");
  }
  else
  {
    sprintf(cgy, "%+5d", (ppm[0] - roll_center));
    Serial.println(cgy);
  }
  Serial.print("pitch: ");
  if (ppm[1] == pitch_center)
  {
    Serial.println("Center");
  }
  else
  {
    sprintf(cgy, "%+5d", (ppm[1] - pitch_center));
    Serial.println(cgy);
  }
  Serial.print("yaw: ");
  if (ppm[3] == yaw_center)
  {
    Serial.println("Center");
  }
  else
  {
    sprintf(cgy, "%+5d", (ppm[3] - yaw_center));
    Serial.println(cgy);
  }
#endif
}

// 來源:pixy範例hello_world
int get_color_info(void)
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
    if (frames % 1 == 0)
    {
      //            sprintf(buf, "Detected %d:\n", blocks);
      //      Serial1.print(buf);
      //            Serial.print(buf);
      for (j = 0; j < blocks; j++)
      {
        //                sprintf(buf, "  block %d: ", j);
        //        Serial1.print(buf);
        //                Serial.print(buf);
        //                pixy.blocks[j].print();
        // 1是紅綠燈紅色,
        if (pixy.blocks[j].signature == 1 || pixy.blocks[j].signature == 2)
        {
          c_idx = pixy.blocks[j].signature - 1;
          our_blocks[c_idx] += 1;
          center_x[c_idx] += pixy.blocks[j].x;
          center_y[c_idx] += pixy.blocks[j].y;
        }
      }
      for (int k = 0; k < colors; k++)
      {
        //        if (our_blocks[k] == 2) {
        center_x[k] = int(center_x[k] / our_blocks[k]);
        center_y[k] = int(center_y[k] / our_blocks[k]);
#if debug == 'B'
        Serial1.println(String("") + "Color " + int(k + 1) + " Center:");
        Serial1.println(String("") + "x: " + center_x[k] + " y: " + center_y[k]);
        Serial1.println("");
#elif debug == 'P'
        Serial.println(String("") + "Color " + int(k + 1) + " Center:");
        Serial.println(String("") + "x: " + center_x[k] + " y: " + center_y[k]);
        Serial.println("");
#endif

        return our_blocks[0];
      }
    }
  }
}

//====================================================PPM產生
ISR(TIMER1_COMPA_vect)
{
  static boolean state = true;

  TCNT1 = 0;

  if (state)
  { //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else
  { //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    digitalWrite(sigPin, !onState);
    state = true;

    if (cur_chan_numb >= chanel_number)
    {
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen; //
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else
    {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }
  }
}

void alt_pid(void)
{
  error = set_d - sonar_cm;
  s += error;
  control = kp * error + ki * s + kd * (error - pre_e);
  pre_e = error;
  new_speed = int(n_speed + control);
  if (new_speed >= 1487)
  {
    new_speed = 1487;
  }
  if (new_speed <= 1478)
  {
    new_speed = 1478;
  }
  ppm_value = new_speed;
}
