#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

int in1 = 5, in2 = 6; // A2, B1 전진 A1, B2 후진
int in3 = 9, in4 = 10; // 6번, 10번 전진
int ENB = 11, FNB = 3;

bool runningCommand1 = false;

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(FNB, OUTPUT);
  Serial.begin(9600); // 시리얼 통신 시작

  // if (!lox.begin()) {
  //   Serial.println(F("Failed to boot VL53L0X"));
  //   while (1) {
  //     delay(100);
  //   }
  // }
}

void loop() {
  // VL53L0X_RangingMeasurementData_t measure;
    
  // lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  // int distance = measure.RangeMilliMeter;

  // if (runningCommand1 && distance < 60) {
  //   stop();
  //   runningCommand1 = false;
  // }

  if (Serial.available() > 0) {
    char command = Serial.read(); // 명령 수신
    
    // if(distance < 50){
    //   left();
    //   if (distance > 50)
    //   delay(2000);
    //   stop();
    // }

    switch (command) {
      case 'F': // 왼쪽회전
        left();
        break;

      case 'B': // 후진
        right();
        break;

      case 'S': // 정지
        runningCommand1 = false;
        stop();
        break;
      
      case '1':
        runningCommand1 = false;
        right();
        delay(300);
        runningCommand1 = true;
        break;
        
      case '2':
        right();
        delay(8000);
        stop();
        break;
    }
  }
}

void left() { //왼쪽 회전
  analogWrite(ENB, 200);
  analogWrite(FNB, 200);

  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH); 
  digitalWrite(in3, HIGH); 
  digitalWrite(in4, LOW);
}

void right() { //오른쪽 회전
  analogWrite(ENB, 200);
  analogWrite(FNB, 200);

  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW); 
  digitalWrite(in3, LOW); 
  digitalWrite(in4, HIGH);
}

void stop() { //정지
  analogWrite(ENB, 250);
  analogWrite(FNB, 250);

  digitalWrite(in1, LOW); 
  digitalWrite(in2, LOW); 
  digitalWrite(in3, LOW); 
  digitalWrite(in4, LOW);
}