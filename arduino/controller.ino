#include <string.h>
#define MAX_SPEED 255 //từ 0-255
//Khai báo động cơ
int M1[2] = {7, 6};
int M2[2] =  {3, 2};
int M3[2] =  {5, 4};
int M4[2] =  {9, 8};
int idle[4] = {0, 0, 0, 0};
// Khai báo leds
#define ROWS  4
#define COLLUM  9
byte LED_a[COLLUM] = {A0, A1, A2, A3, A4, A5, A6, A7, A8};
int LED_ROWS[ROWS] = {28, 22, 26, 24};
int LED_values[ROWS][COLLUM];
void setup() {
  pinMode(M1[0], OUTPUT);
  pinMode(M1[1], OUTPUT);
  pinMode(M2[0], OUTPUT);
  pinMode(M2[1], OUTPUT);
  pinMode(M3[0], OUTPUT);
  pinMode(M3[1], OUTPUT);
  pinMode(M4[0], OUTPUT);
  pinMode(M4[1], OUTPUT);
  pinMode(LED_ROWS[0], OUTPUT);
  pinMode(LED_ROWS[1], OUTPUT);
  pinMode(LED_ROWS[2], OUTPUT);
  pinMode(LED_ROWS[3], OUTPUT);
  digitalWrite(LED_ROWS[0], LOW);
  digitalWrite(LED_ROWS[1], LOW);
  digitalWrite(LED_ROWS[2], LOW);
  digitalWrite(LED_ROWS[3], LOW);
  move(idle);
  Serial.begin(115200);
}
void loop() {
  // Scan LED
  LED_scan();
  send_LEDs_status();
  if (Serial.available()){ //Nếu có tín hiệu từ Pi
    String message = Serial.readStringUntil('/');
    char *buf = message.c_str();
    char *cmd = strtok(buf, " ");
    Serial.println(cmd);
    if (strcmp(cmd, "MOV")==0){
      Serial.println("DO MOVE");
      int speeds[4];
      for(int i = 0; i<4; i++){
        char *speed_str = strtok(NULL, " ");
        speeds[i] = atoi(speed_str);
        Serial.println(speeds[i]);
        }
      move(speeds);
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
void LED_scan(){
    for (int i = 0; i< ROWS; i++){
      digitalWrite(LED_ROWS[i], HIGH);
      delay(1);
      for (int j =0; j< COLLUM; j++){
        int value = analogRead(LED_a[j]);
        LED_values[i][j] = value;
          }
      digitalWrite(LED_ROWS[i], LOW);
      delay(1);
    }
    
}
void send_LEDs_status(){
  char cmd[] = "LEDs";
  char message[1024];
  snprintf(message, sizeof message, "%s", cmd);
  for (int i = 0; i< ROWS; i++){
    for (int j=0; j< COLLUM; j++){
      char value[4];
      sprintf(value, "%d", LED_values[i][j]);
      char space[] = " ";
      snprintf(message, sizeof message, "%s%s%s", message, space, value);
      }
    }
  Serial.println(message);
  }
