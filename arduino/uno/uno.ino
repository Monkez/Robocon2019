#include <string.h>
#define MAX_SPEED 255 //từ 0-255
//Khai báo động cơ
int M1[2] = {5, 4};
int M3[2] =  {11, 10};
int M2[2] =  {9, 8};
int M4[2] =  {3, 2};
int idle[4] = {0, 0, 0, 0};
void setup() {
  pinMode(M1[0], OUTPUT);
  pinMode(M1[1], OUTPUT);
  pinMode(M2[0], OUTPUT);
  pinMode(M2[1], OUTPUT);
  pinMode(M3[0], OUTPUT);
  pinMode(M3[1], OUTPUT);
  pinMode(M4[0], OUTPUT);
  pinMode(M4[1], OUTPUT);
  move(idle);
  Serial.begin(115200);
}
void loop() {
  if (Serial.available()){ //Nếu có tín hiệu từ Pi
    String message = Serial.readStringUntil('/');
    char *buf = message.c_str();
    char *cmd = strtok(buf, " ");
    if (strcmp(cmd, "MOV")==0){
      int speeds[4];
      for(int i = 0; i<4; i++){
        char *speed_str = strtok(NULL, " ");
        speeds[i] = atoi(speed_str);
        }
      move(speeds);
      }
   if (strcmp(cmd, "WHO")==0){
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
