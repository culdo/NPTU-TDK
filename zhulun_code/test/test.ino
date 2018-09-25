#include <SPI.h>  
#define chanel_number 8  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 7  //set PPM signal output pin on the arduino


//////////////////////////////////////////////////////////////////
double ch1;
double ch2;
double ch3;
double ch4;
double ch5;
/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];
void setup(){  
  //initiallize default ppm values
   Serial.begin(115200);
//  for(int i=0; i<chanel_number; i++){
//    ppm[i]= default_servo_value;
//  }
  pinMode(6,INPUT);
  pinMode(5,INPUT);
  pinMode(4,INPUT);
  pinMode(3,INPUT);
  pinMode(2,INPUT);
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
void loop(){ 
// ch5=pulseIn(2,HIGH);//飛行模式  
//  if( ch5<1400){//遙控模式
  ch1=pulseIn(6,HIGH);//roll
 ch2=pulseIn(5,HIGH);//pitch
 ch3=pulseIn(4,HIGH);//油門
 ch4=pulseIn(3,HIGH);//YAW
 ch5=pulseIn(2,HIGH);//mode
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
  delay(500);
  ppm[0]=ch1;
  ppm[1]=ch2;
  ppm[2]=ch3;
  ppm[3]=ch4;
  ppm[4]=ch5; 
//}  
//  else if(ch5>1400&&ch5<1600){//任務模式 /           
//    PIXYSEE();  
//    if(x<150){
//    ch2=pulseIn(5,HIGH);//線左
//    track(ch2 , 1600, 1500);  
//    }
//     else if(x>=150&&x<=170){//線中
//     ch2=pulseIn(5,HIGH);//pitch    
//     track(ch2 , 1500, 1500);  
//     }
//     else{
//     ch2=pulseIn(5,HIGH);//線右    
//     track(ch2 , 1400, 1500);   
//     }
//                                                
//  }
//=========================================================================================     
//   else{//救機降落
//  ch1=pulseIn(6,HIGH);//roll
//  ch2=pulseIn(5,HIGH);//pitch
//  ch3=pulseIn(4,HIGH);//油門
//  ch4=pulseIn(3,HIGH);//YAW 
//  ch5=pulseIn(2,HIGH);//mode
//  ppm[0]=ch1;
//  ppm[1]=ch2;
//  ppm[2]=ch3;
//  ppm[3]=ch4;
//  ppm[4]=1800;
//  }
}
//====================================================PPM產生
ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

