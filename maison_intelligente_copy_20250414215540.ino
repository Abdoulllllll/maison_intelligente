#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>

// === Paramètres étudiants ===
const char* NUM_ETUDIANT = "2412433";

// === LCD I2C ===
LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long lastLcdUpdate = 0;
const unsigned long lcdUpdateInterval = 100;   // Délai de rafraîchissement LCD (ms)

// === Moteur pas-à-pas ===
#define MOTOR_INTERFACE_TYPE 4
#define IN1 31
#define IN2 33
#define IN3 35
#define IN4 37
AccelStepper moteur(MOTOR_INTERFACE_TYPE, IN1, IN3, IN2, IN4);

const int ANGLE_MIN = 10;                      // Angle minimum
const int ANGLE_MAX = 170;                     // Angle maximum
const int ANGLE_INITIAL = 90;                  // Angle de départ
const int STEPS_PAR_180 = 2048;                // Pas pour 180 degrés

int angleCible = ANGLE_INITIAL;
int angleCourant = ANGLE_INITIAL;
bool moteurActif = false;

// === Capteur ultrason ===
const int pinTrig = 9;
const int pinEcho = 8;
long distance = 0;
unsigned long lastDistanceRead = 0;
const unsigned long distanceInterval = 50;     // Fréquence de lecture distance (ms)

const float VITESSE_SON_CM_US = 0.034;         // Vitesse du son en cm/us
const int TIMEOUT_ECHO = 30000;                // Timeout pulseIn (µs)
const int DISTANCE_MIN_VALID = 2;              // Distance minimale valide
const int DISTANCE_MAX_VALID = 400;            // Distance maximale valide

// === Alarme ===
const int pinBuzzer = 26;
const int pinLedRouge = 24;
const int pinLedBleue = 25;

bool alarmeActive = false;
unsigned long lastGyroBlink = 0;
unsigned long lastAlarmeDetection = 0;
bool ledState = false;

// === Seuils système ===
const int DIST_TROP_PRES = 15;
const int DIST_ZONE_MIN = 30;
const int DIST_ZONE_MAX = 60;
const int TEMPS_ALARME = 3000;        // Durée pendant laquelle l'alarme reste active (ms)
const int CLIGNO_INTERVAL = 100;      // Intervalle de clignotement LED

// === Série ===
unsigned long lastSerialTime = 0;
const unsigned long serialInterval = 200;

// === États système ===
enum Etat { TROP_PRES, TROP_LOIN, AUTO };
Etat etatSysteme = TROP_LOIN;

void setup() {
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print(NUM_ETUDIANT);
  lcd.setCursor(0, 1); lcd.print("Labo 4B");
  delay(2000);
  lcd.clear();

  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinLedRouge, OUTPUT);
  pinMode(pinLedBleue, OUTPUT);

  moteur.setMaxSpeed(500);
  moteur.setAcceleration(100);
  moteur.setSpeed(200);
  moteur.setCurrentPosition(0);
  moteur.moveTo(0);
  desactiverMoteur();
}

void loop() {
  lireDistance();
  mettreAJourEtat();
  gererAlarme();
  gererMoteur();
  afficherLCD();
  afficherSerial();
}

void lireDistance() {
  if (millis() - lastDistanceRead >= distanceInterval) {
    digitalWrite(pinTrig, LOW); delayMicroseconds(2);
    digitalWrite(pinTrig, HIGH); delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);

    long duree = pulseIn(pinEcho, HIGH, TIMEOUT_ECHO);
    long d = duree * VITESSE_SON_CM_US / 2;

    if (d >= DISTANCE_MIN_VALID && d <= DISTANCE_MAX_VALID) {
      distance = d;
    }
    lastDistanceRead = millis();
  }
}

void mettreAJourEtat() {
  if (distance <= DIST_TROP_PRES) {
    etatSysteme = TROP_PRES;
  } else if (distance >= DIST_ZONE_MAX) {
    etatSysteme = TROP_LOIN;
  } else {
    etatSysteme = AUTO;
    angleCible = map(distance, DIST_ZONE_MIN, DIST_ZONE_MAX, ANGLE_MIN, ANGLE_MAX);
    angleCible = constrain(angleCible, ANGLE_MIN, ANGLE_MAX);
  }
}

void gererMoteur() {
  if (etatSysteme == AUTO && angleCible != angleCourant) {
    activerMoteur();
    int ciblePas = map(angleCible, 0, 180, -STEPS_PAR_180 / 2, STEPS_PAR_180 / 2);
    moteur.moveTo(ciblePas);
    angleCourant = angleCible;
  }

  if (moteur.distanceToGo() != 0) {
    moteur.run();
  } else if (moteurActif) {
    desactiverMoteur();
  }
}

void activerMoteur() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  moteurActif = true;
}

void desactiverMoteur() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  moteurActif = false;
}

void gererAlarme() {
  unsigned long maintenant = millis();

  if (distance <= DIST_TROP_PRES) {
    lastAlarmeDetection = maintenant;
    alarmeActive = true;
    digitalWrite(pinBuzzer, HIGH);
  }

  if (alarmeActive && (maintenant - lastAlarmeDetection >= TEMPS_ALARME)) {
    digitalWrite(pinBuzzer, LOW);
    digitalWrite(pinLedRouge, LOW);
    digitalWrite(pinLedBleue, LOW);
    alarmeActive = false;
    return;
  }

  if (alarmeActive && (maintenant - lastGyroBlink >= CLIGNO_INTERVAL)) {
    ledState = !ledState;
    digitalWrite(pinLedRouge, ledState ? HIGH : LOW);
    digitalWrite(pinLedBleue, ledState ? LOW : HIGH);
    lastGyroBlink = maintenant;
  }
}

void afficherLCD() {
  if (millis() - lastLcdUpdate >= lcdUpdateInterval) {
    lcd.setCursor(0, 0);
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print("cm   ");

    lcd.setCursor(0, 1);
    if (etatSysteme == TROP_PRES) lcd.print("Obj : Trop pres ");
    else if (etatSysteme == TROP_LOIN) lcd.print("Obj : Trop loin ");
    else {
      lcd.print("Obj : ");
      lcd.print(angleCible);
      lcd.print(" deg ");
    }

    lastLcdUpdate = millis();
  }
}

void afficherSerial() {
  if (millis() - lastSerialTime >= serialInterval) {
    Serial.print("etd:");
    Serial.print(NUM_ETUDIANT);
    Serial.print(", dist:");
    Serial.print(distance);
    Serial.print("cm, angle:");
    Serial.println(angleCible);
    lastSerialTime = millis();
  }
}
