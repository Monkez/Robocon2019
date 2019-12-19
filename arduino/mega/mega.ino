#include <string.h>
#define MAX_SPEED 255 //từ 0-255
//Khai báo động cơ
// Khai báo leds
#define ROWS  4
#define COLLUM  9
byte LED_a[COLLUM] = {A0, A1, A2, A3, A4, A5, A6, A7, A8};
int LED_ROWS[ROWS] = {28, 22, 26, 24};
int LED_values[ROWS][COLLUM];
void setup() {
  pinMode(LED_ROWS[0], OUTPUT);
  pinMode(LED_ROWS[1], OUTPUT);
  pinMode(LED_ROWS[2], OUTPUT);
  pinMode(LED_ROWS[3], OUTPUT);
  digitalWrite(LED_ROWS[0], LOW);
  digitalWrite(LED_ROWS[1], LOW);
  digitalWrite(LED_ROWS[2], LOW);
  digitalWrite(LED_ROWS[3], LOW);
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
    if (strcmp(cmd, "WHO")==0){
      Serial.println("MEGA");
      }
    if (strcmp(cmd, "PUSH")==0){
      Serial.println("PULL");
      }
  }

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
