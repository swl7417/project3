#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(9600);

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1){
      delay(100);
    }
  }
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == '1') {
      if (measure.RangeStatus != 4){  // phase failures have incorrect dat
      int distance = measure.RangeMilliMeter;
      Serial.println(distance);
      } 
      else{
        Serial.println("0");
      }
//    delay(2000);
    } 
  }
}
//  if (measure.RangeStatus != 4){  // phase failures have incorrect dat
//    int distance = measure.RangeMilliMeter;
//    Serial.println(distance);
//  } 
//  else{
//    Serial.println("0");
//  }
//    
//  delay(1000);
