#include <Arduino.h>
#include <TFLI2C.h>
#include <Wire.h>

//--
TFLI2C lidar;

//-- Variables

// lidar measurements
int16_t distanceCM;

// lidar adddress
const uint8_t lidarAddress = 0x10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (lidar.getData(distanceCM, lidarAddress)) {
    Serial.print("Distance (cm): ");
    Serial.println(distanceCM);
  }

  else {
    Serial.println("Error reading liDAR");
  }
  
  delay(20);
}