#include <string.h>
#include <Servo.h>

#define MAX_SPEED 255 //từ 0-255
//Khai báo động cơ
int M1[2] = {5, 4};
int M2[2] =  {6, 7};
int M3[2] =  {11, 10};
int M4[2] =  {3, 2};
int M5[2] = {8, 9};
int M6[2] = {12, 13};
int idle[4] = {0, 0, 0, 0};
byte servo_pins[4] = {A0, A1, A2, A3};
Servo servo[4];
void setup() {
  pinMode(M1[0], OUTPUT);
  pinMode(M1[1], OUTPUT);
  pinMode(M2[0], OUTPUT);
  pinMode(M2[1], OUTPUT);
  pinMode(M3[0], OUTPUT);
  pinMode(M3[1], OUTPUT);
  pinMode(M4[0], OUTPUT);
  pinMode(M4[1], OUTPUT);
  pinMode(M5[0], OUTPUT);
  pinMode(M5[1], OUTPUT);
  pinMode(M6[0], OUTPUT);
  pinMode(M6[1], OUTPUT);
  for (int i=0;i<4;i++){
    servo[i].attach(servo_pins[i]);
    }
 
  move(idle);
  Serial.begin(250000);
}
void loop() {
  if (Serial.available()){ //Nếu có tín hiệu từ Pi
    String message = Serial.readStringUntil('/');
    const char *buf = message.c_str();
    char *cmd = strtok(buf, " ");
    if (strcmp(cmd, "MOV")==0){
      int speeds[4];
      for(int i = 0; i<4; i++){
        char *speed_str = strtok(NULL, " ");
        speeds[i] = atoi(speed_str);
        }
      move(speeds);
      }
   if (strcmp(cmd, "SER")==0){
      char *stt = strtok(NULL, " ");
      int ser = atoi(stt);
      char *angle = strtok(NULL, " ");
      int ser_angle = atoi(angle);
      servo[ser].write(180 - ser_angle);
      delay(15);
   }

   if (strcmp(cmd, "SHOT")==0){
      char *stt = strtok(NULL, " ");
      int speed = atoi(stt);
      if (speed>0){
        digitalWrite(M5[0], HIGH);
        digitalWrite(M5[1], LOW);
        } else{
          digitalWrite(M5[0], LOW);
        digitalWrite(M5[1], LOW);
          }
      
   }

   if (strcmp(cmd, "FHAND")==0){
      char *stt = strtok(NULL, " ");
      int speed = atoi(stt);
      if (speed>0){
        digitalWrite(M6[0], HIGH);
        digitalWrite(M6[1], LOW);
        }
      if (speed<0){
        digitalWrite(M6[0], LOW);
        digitalWrite(M6[1], HIGH);
        }
      if (speed==0){
        digitalWrite(M6[0], LOW);
        digitalWrite(M6[1], LOW);
        }
   }
   
   if (strcmp(cmd, "WHO")==0){
      delay(0.2);
      Serial.println("UNO");
      }
  }

}
void set_motor(int M[2], int speed) {
  speed = constrain(speed, - MAX_SPEED, MAX_SPEED);//đảm báo giá trị nằm trong một khoảng từ 0 - MAX_SPEED
   if (speed >=0){
    analogWrite(M[0],speed);
    digitalWrite(M[1], LOW);// chân này không có PWM
   } else {
    analogWrite(M[0], 255  + speed);
    digitalWrite(M[1], HIGH);// chân này không có PWM
   }
}
// Điều khiển chuyển động của xe
void move(int speeds[4]){
  set_motor(M1, speeds[0]);
  set_motor(M2, speeds[1]);
  set_motor(M3, speeds[2]);
  set_motor(M4, speeds[3]);
  }
