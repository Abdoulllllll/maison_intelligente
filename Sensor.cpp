#include "Sensor.h"
#include <Arduino.h>

Sensor::Sensor(int trigPin, int echoPin) {
  trig = trigPin;
  echo = echoPin;
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

void Sensor::read() {
  if (millis() - lastRead < interval) return;
  lastRead = millis();

  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);
  distance = duration * 0.034 / 2;
}

long Sensor::getDistance() {
  return distance;
}
