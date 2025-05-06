#include "Alarm.h"
#include <Arduino.h>

Alarm::Alarm(int buzz, int r, int b, int seuil_) {
  buzzer = buzz;
  ledR = r;
  ledB = b;
  seuil = seuil_;
  pinMode(buzzer, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(ledB, OUTPUT);
}

void Alarm::update(long distance) {
  unsigned long now = millis();

  if (distance <= seuil) {
    lastDetect = now;
    active = true;
    digitalWrite(buzzer, HIGH);
  }

  if (active && now - lastDetect >= 3000) {
    active = false;
    digitalWrite(buzzer, LOW);
    digitalWrite(ledR, LOW);
    digitalWrite(ledB, LOW);
  }

  if (active && now - lastBlink >= 100) {
    ledState = !ledState;
    digitalWrite(ledR, ledState);
    digitalWrite(ledB, !ledState);
    lastBlink = now;
  }
}
