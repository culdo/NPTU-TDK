char cmd[4] ;

void setup(){
Serial.begin(9600);
//Serial.setTimeout(10); // 設定為每10毫秒結束一次讀取(數字愈小愈快)
}

void loop(){
//if(Serial.available()){
 Serial.readBytes(cmd,4);
 Serial.println(atoi(cmd));
//}
}
