#include <Arduino.h>

//-- Pins
const uint8_t trigPin = 7;
const uint8_t echoPin = 8;

//-- Variables

// give up waiting for an echo after this long (~4 m round trip)
const unsigned long echoTimeoutUS = 25000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  // a 10us pulse on trig starts a measurement
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // echo stays high for the sound's round-trip time; 0 means no echo
  unsigned long durationUS = pulseIn(echoPin, HIGH, echoTimeoutUS);

  if (durationUS > 0) {
    // sound travels ~0.0343 cm/us, halved for the round trip
    int distanceCM = durationUS * 0.0343 / 2;
    Serial.print("Distance (cm): ");
    Serial.println(distanceCM);
  }

  else {
    Serial.println("Error reading ultrasonic sensor (no echo)");
  }

  // leave time for old echoes to die out before the next ping
  delay(40);
}
