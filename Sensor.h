#ifndef SENSOR_H
#define SENSOR_H

class Sensor {
  int trig, echo;
  long distance = 0;
  unsigned long lastRead = 0;
  const unsigned long interval = 50;

public:
  Sensor(int trigPin, int echoPin);
  void read();
  long getDistance();
};

#endif
