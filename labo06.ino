#include <Wire.h> // Inclure la bibliothèque Wire pour la communication I2C
#include <LiquidCrystal_I2C.h> // Inclure la bibliothèque pour contrôler l'écran LCD I2C
#include <AccelStepper.h> // Inclure la bibliothèque pour contrôler un moteur pas-à-pas
#include <U8g2lib.h> // Inclure la bibliothèque pour gérer l'afficheur MAX7219

// Définir le numéro d'étudiant affiché
const char* NUM_ETUDIANT = "2412433";

// Définir les broches utilisées pour le MAX7219 (Afficheur LED)
#define CLK_PIN 52 // Broche de l'horloge 
#define DIN_PIN 51 // Broche de données 
#define CS_PIN 53  // Broche de sélection du module (Chip Select)

// Initialiser l'afficheur MAX7219 avec les bonnes broches
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(U8G2_R0, CLK_PIN, DIN_PIN, CS_PIN, U8X8_PIN_NONE, U8X8_PIN_NONE);

// Initialiser l'écran LCD avec l'adresse I2C 0x27, 16 colonnes, 2 lignes
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variable pour contrôler le délai de mise à jour du LCD
unsigned long lastLcdUpdate = 0;
const unsigned long lcdUpdateInterval = 100; // Intervalle d'actualisation du LCD (en ms)

// Définir les broches moteur pas-à-pas
#define MOTOR_INTERFACE_TYPE 4 // Type d'interface moteur : 4 fils
#define IN1 31 // Broche IN1 du moteur
#define IN2 33 // Broche IN2 du moteur
#define IN3 35 // Broche IN3 du moteur
#define IN4 37 // Broche IN4 du moteur

// Initialiser le moteur pas-à-pas
AccelStepper moteur(MOTOR_INTERFACE_TYPE, IN1, IN3, IN2, IN4);

// Constantes pour la gestion des angles du moteur
const int ANGLE_MIN = 10; // Angle minimum pour la position du moteur
const int ANGLE_MAX = 170; // Angle maximum
const int STEPS_PAR_180 = 2048; // Nombre de pas moteur pour 180 degrés

// Variables pour l'angle actuel et cible
int angleCible = 90; // Angle de départ visé
int angleCourant = 90; // Angle actuel
bool moteurActif = false; // État moteur actif/inactif

// Broches pour le capteur ultrason
const int pinTrig = 9; // Broche TRIG
const int pinEcho = 8; // Broche ECHO

// Variables pour la mesure de distance
long distance = 0; // Dernière distance mesurée
unsigned long lastDistanceRead = 0; // Temps de dernière lecture
const unsigned long distanceInterval = 50; // Fréquence de mesure (ms)

// Constantes pour la mesure ultrason
const float VITESSE_SON_CM_US = 0.034; // Vitesse du son en cm/us
const int TIMEOUT_ECHO = 30000; // Temps maximum pour attendre un echo

// Broches et variables pour l'alarme
const int pinBuzzer = 26; // Broche du buzzer
const int pinLedRouge = 24; // Broche LED rouge
const int pinLedBleue = 25; // Broche LED bleue
bool alarmeActive = false; // Indique si l'alarme est active
unsigned long lastGyroBlink = 0; // Dernier clignotement
unsigned long lastAlarmeDetection = 0; // Dernière détection d'intrusion
bool ledState = false; // État ON/OFF de la LED

// Seuils pour la détection
int DIST_TROP_PRES = 15; // Seuil trop près pour déclencher l'alarme
int limInf = 30; // Limite inférieure de détection moteur
int limSup = 60; // Limite supérieure
const int TEMPS_ALARME = 3000; // Durée de l'alarme (ms)
const int CLIGNO_INTERVAL = 100; // Intervalle de clignotement LED (ms)

// Définir les états possibles du système
enum Etat { TROP_PRES, TROP_LOIN, AUTO };
Etat etatSysteme = TROP_LOIN; // État initial

// Définir les états d'affichage MAX7219
enum EtatAffichage { AUCUN, CONFIRMATION, ERREUR, INCONNU };
EtatAffichage etatAffichageMAX = AUCUN; // État initial affichage
unsigned long affichageStart = 0; // Début du timer d'affichage

// ================= SETUP =================
void setup() {
  Serial.begin(115200); // Initialiser la communication série

  // Initialiser les broches capteur ultrason
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);

  // Initialiser les broches LED et buzzer
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinLedRouge, OUTPUT);
  pinMode(pinLedBleue, OUTPUT);

  // Initialiser moteur
  moteur.setMaxSpeed(500);
  moteur.setAcceleration(100);
  moteur.setSpeed(200);
  moteur.setCurrentPosition(0);

  // Initialiser LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print(NUM_ETUDIANT); // Afficher numéro étudiant
  lcd.setCursor(0, 1); lcd.print("Labo 6"); // Afficher titre labo
  delay(2000); // Pause de 2 secondes
  lcd.clear(); // Effacer écran

  // Initialiser MAX7219
  u8g2.begin();
  u8g2.setContrast(5);
  u8g2.setFont(u8g2_font_4x6_tr);
}

// ================= LOOP =================
void loop() {
  lireDistance(); // Lire la distance ultrason
  mettreAJourEtat(); // Mettre à jour l'état selon la distance
  gererAlarme(); // Gérer le buzzer et les LED
  gererMoteur(); // Gérer le déplacement du moteur
  afficherLCD(); // Afficher sur LCD
  afficherMAX7219(); // Afficher sur MAX7219
  gererCommandes(); // Lire et traiter commandes série
}

// ================= FONCTIONS =================

// Lire la distance du capteur ultrason
void lireDistance() {
  if (millis() - lastDistanceRead >= distanceInterval) { // Lire si délai écoulé
    digitalWrite(pinTrig, LOW); delayMicroseconds(2); // Début trigger
    digitalWrite(pinTrig, HIGH); delayMicroseconds(10); // Envoi d'une impulsion
    digitalWrite(pinTrig, LOW); // Fin trigger

    long duree = pulseIn(pinEcho, HIGH, TIMEOUT_ECHO); // Mesure de l'écho
    distance = duree * VITESSE_SON_CM_US / 2; // Conversion durée -> distance
    lastDistanceRead = millis(); // Mise à jour du dernier temps
  }
}

// Déterminer l'état du système selon la distance
void mettreAJourEtat() {
  if (distance <= DIST_TROP_PRES) etatSysteme = TROP_PRES; // Trop proche
  else if (distance >= limSup) etatSysteme = TROP_LOIN; // Trop loin
  else { // Entre les deux : automatique
    etatSysteme = AUTO;
    angleCible = map(distance, limInf, limSup, ANGLE_MIN, ANGLE_MAX);
    angleCible = constrain(angleCible, ANGLE_MIN, ANGLE_MAX);
  }
}

// Contrôler le moteur pas-à-pas
void gererMoteur() {
  if (etatSysteme == AUTO && angleCible != angleCourant) { // Si L'angle a changé
    activerMoteur(); // Activer les bobines
    int ciblePas = map(angleCible, 0, 180, -STEPS_PAR_180 / 2, STEPS_PAR_180 / 2);
    moteur.moveTo(ciblePas); // Aller vers la nouvelle position
    angleCourant = angleCible; // Mettre à jour
  }
  if (moteur.distanceToGo() != 0) moteur.run(); // Continuer le mouvement
  else if (moteurActif) desactiverMoteur(); // Éteindre bobines si arrivé
}

// Allumer toutes les bobines du moteur
void activerMoteur() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  moteurActif = true;
}

// Éteindre toutes les bobines
void desactiverMoteur() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  moteurActif = false;
}

// Gérer l'alarme sonore et les LEDs
void gererAlarme() {
  unsigned long maintenant = millis(); // Temps actuel

  if (distance <= DIST_TROP_PRES) { // Détecte objet trop proche
    lastAlarmeDetection = maintenant;
    alarmeActive = true;
    digitalWrite(pinBuzzer, HIGH);
  }

  if (alarmeActive && (maintenant - lastAlarmeDetection >= TEMPS_ALARME)) { // Temps écoulé
    digitalWrite(pinBuzzer, LOW);
    digitalWrite(pinLedRouge, LOW);
    digitalWrite(pinLedBleue, LOW);
    alarmeActive = false;
  }

  if (alarmeActive && (maintenant - lastGyroBlink >= CLIGNO_INTERVAL)) { // Clignoter LED
    ledState = !ledState;
    digitalWrite(pinLedRouge, ledState);
    digitalWrite(pinLedBleue, !ledState);
    lastGyroBlink = maintenant;
  }
}

// Mise à jour de l'affichage LCD
void afficherLCD() {
  if (millis() - lastLcdUpdate >= lcdUpdateInterval) {
    lcd.setCursor(0, 0);
    lcd.print("Dist: "); lcd.print(distance); lcd.print("cm   ");
    lcd.setCursor(0, 1);
    if (etatSysteme == TROP_PRES) lcd.print("Obj : Trop pres ");
    else if (etatSysteme == TROP_LOIN) lcd.print("Obj : Trop loin ");
    else {
      lcd.print("Obj : "); lcd.print(angleCible); lcd.print(" deg ");
    }
    lastLcdUpdate = millis();
  }
}

// Affichage des symboles sur le MAX7219
void afficherMAX7219() {
  if (etatAffichageMAX == AUCUN) return; // Si rien à afficher

  if (millis() - affichageStart >= 3000) { // Effacer après 3 secondes
    etatAffichageMAX = AUCUN;
    u8g2.clear();
    return;
  }

  u8g2.clearBuffer();
  if (etatAffichageMAX == CONFIRMATION) { // bon
    u8g2.drawLine(1, 6, 3, 8);
    u8g2.drawLine(3, 8, 7, 0);
  } else if (etatAffichageMAX == ERREUR) { // interdit signe
    u8g2.drawCircle(4, 4, 3);
    u8g2.drawLine(2, 2, 6, 6);
  } else if (etatAffichageMAX == INCONNU) { // x
    u8g2.drawLine(1, 1, 6, 6);
    u8g2.drawLine(6, 1, 1, 6);
  }
  u8g2.sendBuffer();
}

// Lire les commandes envoyées par le moniteur série
void gererCommandes() {
  if (!Serial.available()) return;

  String tampon = Serial.readStringUntil('\n');
  tampon.trim(); tampon.toLowerCase();

  if (tampon == "g_dist") { // Commande de lecture distance
    Serial.println(distance);
    etatAffichageMAX = CONFIRMATION;
  } else if (tampon.startsWith("cfg;alm;")) { // Modifier seuil alarme
    DIST_TROP_PRES = tampon.substring(8).toInt();
    etatAffichageMAX = CONFIRMATION;
  } else if (tampon.startsWith("cfg;lim_inf;")) { // Modifier limite inférieure
    int val = tampon.substring(12).toInt();
    if (val < limSup) { limInf = val; etatAffichageMAX = CONFIRMATION; }
    else etatAffichageMAX = ERREUR;
  } else if (tampon.startsWith("cfg;lim_sup;")) { // Modifier limite supérieure
    int val = tampon.substring(12).toInt();
    if (val > limInf) { limSup = val; etatAffichageMAX = CONFIRMATION; }
    else etatAffichageMAX = ERREUR;
  } else { // Commande non reconnue
    etatAffichageMAX = INCONNU;
  }
  affichageStart = millis(); // Démarrer le chrono d'affichage
}
