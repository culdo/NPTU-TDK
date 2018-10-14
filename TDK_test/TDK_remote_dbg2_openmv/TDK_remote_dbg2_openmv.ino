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
#define roll_center 1462  //好螺旋1462,1485
#define pitch_forward 1440 //好螺旋1419,1447
#define pitch_center 1447
#define yaw_center 1484   //1494,1484
//#include <SoftwareSerial.h>   // 引用程式庫
#include <Pixy.h>
#define colors 2
#define origin_x 159
#define origin_y 99
#define roll_pwm pwm[0]
#define pitch_pwm pwm[1]
#define throttle_pwm pwm[2]
#define yaw_pwm pwm[3]
#define mode_pwm pwm[4]

//debug選項: false=關閉debug 'P'=電腦PC 'B'=藍芽Bluetooth
#define debug 'P'

// 定義連接藍牙模組的序列埠
//SoftwareSerial 'B'(3, 12); // 接收腳, 傳送腳
char val; // 儲存接收資料的變數

//#define debug false

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
// This is the main Pixy object


unsigned long now;
unsigned long start, timer, before, before_op = 0;
unsigned long interval = 0;
bool is_takeoff = false;
bool is_land = false;
bool is_sonic_fly = false;
bool is_get_red = false;
bool is_error = false;
bool is_high = false;
bool open_pid=false;
bool is_get_yellow = false;
//bool switched = false;

//////////////////////////////////////////////////////////////////
//int mode_pwm;
/*this array holds the servo values for the ppm signal
  change theese values in your code (usually servo values move between 1000 and 2000)*/
int pwm[chanel_number];
int ppm_value;

int ppm_value1;
int updated_times = 0;
int sonar_cm = 0;
//openmv
char cmd[4];
char garbage[4];
int openmv;
int c = 0;
float control;
float n_speed = 1482; //1484
int new_speed;
float pre_e = 0;
int set_d = 75;
float error;
float s = 0;
float kp = 0.16; //0.1
float ki = 0;
float kd = 0.4; //0.25
int y;
int s1=0;
Pixy pixy;
//float kp1= 0.1;//0.1
//float ki1 = 0;
//float kd1 = 0.2;//0.3
//int error_pwm=2555;

void setup()
{

  //initiallize default ppm values
  // 設為115200平滑接收監控訊息
//  Serial2.begin(115200);
//#if debug == 'B'
//  Serial1.begin(115200);
//#elif debug == 'P'
//  //藍芽鮑率
//  Serial.begin(115200);
//#endif

  for (int i = 0; i < chanel_number; i++)
  {
    pwm[i] = default_servo_value;
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
//#if debug != false
//  if ((millis() - interval) > 400)
//  {
//    print_status();
//    //    get_color_info();
//    interval = millis();
//  }
//#endif

  if (throttle_pwm >= 1520 || is_error == true)
  {
    is_error = true;
    land_mode();
  }
  else
  {
    mode_pwm = pulseIn(ch7_pin, HIGH); //飛行模式
    //    mode_pwm = mode_pwm;                 //mode

    if (mode_pwm < 1300 && (!is_sonic_fly))
    // if (mode_pwm < 1300) //遙控模式
    {
      if (!is_sonic_fly)
        rc_mode();
      //      else
      //        error_pwm = mode_pwm;
    }
    else if (mode_pwm >= 1300 && mode_pwm < 1600)
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
    if ((pulseIn(ch4_pin, HIGH)) > 1850)
    {
      start = millis();
      yaw_pwm = 1960;
    }
    else
    {
      yaw_pwm = pulseIn(ch4_pin, HIGH); //yaw
    }
  }
  // 為了Pixy讓出三隻plueIn腳(Uno板)
  roll_pwm = pulseIn(ch1_pin, HIGH);           //roll
  pitch_pwm = pulseIn(ch2_pin, HIGH);          //pitch
  throttle_pwm = pulseIn(ch3_pin, HIGH) - 150; //油門
  //  mode_pwm = pulseIn(ch7_pin, HIGH);       //mode
}

void mission_mode(void)
{
  int rel_x, rel_y;
  unsigned long buf;

  is_land = false;
  if (is_takeoff == false)
  {
    start = millis();
    is_takeoff = true;
    // ppm_value = 1455;
    ppm_value = 1475;
    roll_pwm = roll_center;   //1500,1460
    pitch_pwm = pitch_forward; //1435,1450
    yaw_pwm = yaw_center;
   throttle_pwm = ppm_value;
  }
  else
  {
    now = millis();
    //    if ((now - start) <= 5000)//5000
    //    {
    //      roll_pwm = roll_center;  //1500,1460
    //      pitch_pwm = pitch_forward; //1435,1450
    //      throttle_pwm = 1420;         //1465
    //      yaw_pwm = yaw_center;   //1500
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

    if (sonar_cm >= 50 && open_pid == false)
    {
      open_pid = true;
    }
    if (open_pid == true)
    {
      alt_pid();
      get_color_info();

    }
    throttle_pwm = ppm_value; //1440
    if (s1==1) {
    pitch_pwm=1560;
    //delay(100);
    }
    if (s1==0) {
     pitch_pwm=1450;  
    }
//    Serial.println(pitch_pwm);
//    Serial.println(throttle_pwm);
//    Serial.println(open_pid);
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
  //            pitch_pwm = pitch_center;
  //            yaw_pwm = atoi(cmd);
  //          }
  //        }
  //        else if (millis() - before <= 1500) {
  //          yaw_pwm = yaw_center;
  //          pitch_pwm = pitch_center - 10;
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
 // open_pid = false;
  if (is_land == false)
  {
    before = 0;
    roll_pwm = roll_center;
    pitch_pwm = pitch_forward;
    throttle_pwm = ppm_value; //1480;//1450
    yaw_pwm = yaw_center;
    start = millis();
    is_land = true;
  }
  else
  {
    now = millis();
    if (throttle_pwm > 1430)
    {
      if (now - before >= 2750)
      {
        before = millis();
        throttle_pwm -= 5;
      }
    }
    else
    {
      roll_pwm = 1470;
      pitch_pwm = 1900;
      throttle_pwm = 1000;
      yaw_pwm = 1000;
      mode_pwm = 1800;
    }

    if (throttle_pwm <= 1450)
    {
      is_sonic_fly = false;
    }
    //    if (now - start <= 2000) {
    //      roll_pwm = roll_center;
    //      pitch_pwm = pitch_center;
    //      throttle_pwm = 1460;//1450
    //      yaw_pwm = yaw_center;
    //      mode_pwm = 1800;
    //    }
    //    else if ( now - start <= 3000 ) {
    //      roll_pwm = roll_center;
    //      pitch_pwm = pitch_center;
    //      throttle_pwm = 1455;//1250
    //      yaw_pwm = yaw_center;
    //      //        mode_pwm = 1800;
    //    }
    //    else if (now - start <= 4000 ) {
    //      roll_pwm = roll_center;
    //      pitch_pwm = pitch_center;
    //      throttle_pwm = 1450;//1250
    //      yaw_pwm = yaw_center;
    //      //        mode_pwm = 1800;
    //    }
    //    else {
    //      roll_pwm = 1470;
    //      pitch_pwm = 1900;
    //      throttle_pwm = 1000;
    //      yaw_pwm = 1000;
    //      mode_pwm = 1800;
    //    }
    //  }
  }
}

// 來源:pixy範例hello_world
void get_color_info(void)
{
     int i;
     uint16_t blocks;
   // grab blocks!
  blocks = pixy.getBlocks();

  // If there are detect blocks, print them!
  if (blocks)
  {
//     if (blocks==1)
//        {
//          y=pixy.blocks[0].y;   
//        }
//    else if (blocks==2)
//    {
//      y=int((pixy.blocks[0].y+pixy.blocks[1].y)/2);
//    }

//      ppm_value1=1550;
      s1=1;
     delay(20);
//    if (y<89) {
//      ppm_value1=1440;
//    //  s=1;
//     }
//     else if (y>=89 && y<=109) {
//    ppm_value1=1442;
//    //  s=2;
//     }
//     else if (y>109) {
//     ppm_value1=1447;
//   //   s=3;
//     }
     
  }  
  else {
//   ppm_value1=1440;
    s1=0;
     delay(20);
  }
}

void print_status()
{
  // ==========================只能選擇一種debug方式其他Serial請comment掉!!!==========================
  char cgy[10];
#if debug == 'B'
  //    Serial1.print("ERROR_MODE_pwm: ");
  //    Serial1.println(error_pwm);
  if (mode_pwm < 1300)
  {
    Serial1.println("================Radio Mode================");
  }
  else if (mode_pwm >= 1300 && mode_pwm < 1600)
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
  Serial1.println(mode_pwm);
  Serial1.print("throttle: ");
  Serial1.println(throttle_pwm);
  Serial1.print("roll: ");
  if (roll_pwm == roll_center)
  {
    Serial1.println("Center");
  }
  else
  {
    sprintf(cgy, "%+5d", (roll_pwm - roll_center));
    Serial1.println(cgy);
  }
  Serial1.print("pitch: ");
  if (pitch_pwm == pitch_forward)
  {
    Serial1.println("Forward>>>>");
  }
  else if (pitch_pwm == pitch_center)
  {
    Serial1.println("Center");
  }
  else 
  {
    sprintf(cgy, "%+5d", (pitch_pwm - pitch_center));
    Serial1.println(cgy);
  }
  Serial1.print("yaw: ");
  if (yaw_pwm == yaw_center)
  {
    Serial1.println("Center");
  }
  else
  {
    sprintf(cgy, "%+5d", (yaw_pwm - yaw_center));
    Serial1.println(cgy);
  }
#elif debug == 'P'

  if (mode_pwm < 1300)
  {
    Serial.println("================Radio Mode================");
  }
  else if (mode_pwm >= 1300 && mode_pwm < 1600)
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
  Serial.println(mode_pwm);
  Serial.print("throttle: ");
  Serial.println(throttle_pwm);
  Serial.print("roll: ");
  if (roll_pwm == roll_center)
  {
    Serial.println("Center");
  }
  else
  {
    sprintf(cgy, "%+5d", (roll_pwm - roll_center));
    Serial.println(cgy);
  }
  Serial.print("pitch: ");
  if (pitch_pwm == pitch_forward)
  {
    Serial.println("Forward");
  }
  else
  {
    sprintf(cgy, "%+5d", (pitch_pwm - pitch_forward));
    Serial.println(cgy);
  }
  Serial.print("yaw: ");
  if (yaw_pwm == yaw_center)
  {
    Serial.println("Center");
  }
  else
  {
    sprintf(cgy, "%+5d", (yaw_pwm - yaw_center));
    Serial.println(cgy);
  }
#endif
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
      OCR1A = (pwm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + pwm[cur_chan_numb];
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
