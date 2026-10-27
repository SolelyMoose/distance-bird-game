#include <Arduino.h>
#include <TFLI2C.h>
#include <Wire.h>

//--
TFLI2C lidar;

//-- Pins
const uint8_t trigPin = 7;
const uint8_t echoPin = 8;

//-- Variables

// lidar adddress
const uint8_t lidarAddress = 0x10;

// give up waiting for an echo after this long (~4 m round trip)
const unsigned long echoTimeoutUS = 25000;

// returns -1 if the lidar couldn't be read
int readLidarCM() {
  int16_t distanceCM;
  if (lidar.getData(distanceCM, lidarAddress)) {
    return distanceCM;
  }
  return -1;
}

// returns -1 if no echo came back
int readUltrasonicCM() {
  // a 10us pulse on trig starts a measurement
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // echo stays high for the sound's round-trip time; 0 means no echo
  unsigned long durationUS = pulseIn(echoPin, HIGH, echoTimeoutUS);
  if (durationUS == 0) {
    return -1;
  }

  // sound travels ~0.0343 cm/us, halved for the round trip
  return durationUS * 0.0343 / 2;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  // one line with both readings, e.g. "Luna (cm): 34  Sonar (cm): 33"
  Serial.print("Luna (cm): ");
  Serial.print(readLidarCM());
  Serial.print("  Sonar (cm): ");
  Serial.println(readUltrasonicCM());

  // leave time for old echoes to die out before the next ping
  delay(40);
}
