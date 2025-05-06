#include "ViseurAutomatique.h"
#include <AccelStepper.h>
#include <Arduino.h>

// === Pins et objets ===
static int _pinTrig, _pinEcho;
static AccelStepper moteur(AccelStepper::FULL4WIRE, 31, 35, 33, 37);

// === Distance ===
static long distance = 0;
static unsigned long lastDistanceRead = 0;
const unsigned long INTERVAL_DISTANCE = 50;
const int TIMEOUT_ECHO = 30000;
const float VITESSE_SON_CM_US = 0.034;
const int DIST_MIN_VALID = 2;
const int DIST_MAX_VALID = 400;

// === Angle ===
const int ANGLE_MIN = 10;
const int ANGLE_MAX = 170;
const int STEPS_PAR_180 = 2048;
static int angleCourant = 90;
static int angleCible = 90;
static bool moteurActif = false;

// === Zone AUTO ===
const int LIM_INF = 30;
const int LIM_SUP = 60;

// === État système ===
enum Etat { TROP_PRES, TROP_LOIN, AUTO };
static Etat etatSysteme = TROP_LOIN;

void viseur_init(int pinTrig, int pinEcho, int pinIN1, int pinIN2, int pinIN3, int pinIN4) {
  _pinTrig = pinTrig;
  _pinEcho = pinEcho;
  pinMode(_pinTrig, OUTPUT);
  pinMode(_pinEcho, INPUT);

  moteur.setMaxSpeed(500);
  moteur.setAcceleration(100);
  moteur.setSpeed(200);
  moteur.setCurrentPosition(0);
  moteur.moveTo(0);
}

void lireDistance() {
  if (millis() - lastDistanceRead < INTERVAL_DISTANCE) return;
  digitalWrite(_pinTrig, LOW); delayMicroseconds(2);
  digitalWrite(_pinTrig, HIGH); delayMicroseconds(10);
  digitalWrite(_pinTrig, LOW);
  long duree = pulseIn(_pinEcho, HIGH, TIMEOUT_ECHO);
  long d = duree * VITESSE_SON_CM_US / 2;
  if (d >= DIST_MIN_VALID && d <= DIST_MAX_VALID) distance = d;
  lastDistanceRead = millis();
}

void viseur_update() {
  lireDistance();

  // État logique
  if (distance <= 15) {
    etatSysteme = TROP_PRES;
  } else if (distance >= LIM_SUP) {
    etatSysteme = TROP_LOIN;
  } else {
    etatSysteme = AUTO;
    angleCible = map(distance, LIM_INF, LIM_SUP, ANGLE_MIN, ANGLE_MAX);
    angleCible = constrain(angleCible, ANGLE_MIN, ANGLE_MAX);
  }

  // Moteur
  if (etatSysteme == AUTO && angleCible != angleCourant) {
    int ciblePas = map(angleCible, 0, 180, -STEPS_PAR_180 / 2, STEPS_PAR_180 / 2);
    moteur.moveTo(ciblePas);
    angleCourant = angleCible;
  }
}

void viseur_refresh() {
  if (moteur.distanceToGo() != 0) {
    moteur.run();
  }
}

long viseur_distance() {
  return distance;
}

int viseur_angle() {
  return angleCible;
}