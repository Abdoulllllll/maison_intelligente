#ifndef ALARM_H
#define ALARM_H

class Alarm {
  int buzzer, ledR, ledB;
  int seuil;
  bool active = false;
  unsigned long lastDetect = 0;
  unsigned long lastBlink = 0;
  bool ledState = false;

public:
  Alarm(int buzz, int r, int b, int seuil);
  void update(long distance);
};

#endif
