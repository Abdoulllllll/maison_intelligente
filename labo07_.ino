#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <U8g2lib.h>

#include "Sensor.h"
#include "Alarm.h"
#include "PorteAutomatique.h"

#define TRIG_PIN 9
#define ECHO_PIN 8

#define BUZZER_PIN 26
#define LED_ROUGE 24
#define LED_BLEUE 25

#define IN1 31
#define IN2 33
#define IN3 35
#define IN4 37

#define CLK_PIN 52
#define DIN_PIN 51
#define CS_PIN 53

Sensor capteur(TRIG_PIN, ECHO_PIN);
Alarm alarme(BUZZER_PIN, LED_ROUGE, LED_BLEUE, 15);
AccelStepper moteur(4, IN1, IN3, IN2, IN4);
Viseur viseur(&moteur, 10, 170, 2048, 30, 60);

LiquidCrystal_I2C lcd(0x27, 16, 2);
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(U8G2_R0, CLK_PIN, DIN_PIN, CS_PIN, U8X8_PIN_NONE, U8X8_PIN_NONE);

const char* NUM_ETUDIANT = "2412433";
unsigned long lastLcdUpdate = 0;
const unsigned long lcdInterval = 100;

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print(NUM_ETUDIANT);
  lcd.setCursor(0, 1); lcd.print("Labo 7");
  delay(2000);
  lcd.clear();

  u8g2.begin();
  u8g2.setContrast(5);
  u8g2.setFont(u8g2_font_4x6_tr);
}

void loop() {
  capteur.read();
  porte.update(capteur.getDistance());
  alarme.update(capteur.getDistance());

  porte.run();

  afficherLCD();
  afficherMAX7219();
}

void afficherLCD() {
  if (millis() - lastLcdUpdate < lcdInterval) return;
  lastLcdUpdate = millis();

  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(capteur.getDistance());
  lcd.print("cm   ");

  lcd.setCursor(0, 1);
  if (capteur.getDistance() < 30) lcd.print("Obj : Trop pres ");
  else if (capteur.getDistance() > 60) lcd.print("Obj : Trop loin ");
  else {
    lcd.print("Obj : ");
    lcd.print(porte.getAngle());
    lcd.print(" deg ");
  }
}

void afficherMAX7219() {
  static unsigned long startTime = 0;
  static int mode = 0;

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toLowerCase();
    mode = 0;

    if (cmd == "g_dist") {
      Serial.println(capteur.getDistance());
      mode = 1;
    } else {
      mode = 2;
    }

    startTime = millis();
  }

  if (mode == 0 || millis() - startTime > 3000) return;

  u8g2.clearBuffer();
  if (mode == 1) { // ✔
    u8g2.drawLine(1, 6, 3, 8);
    u8g2.drawLine(3, 8, 7, 0);
  } else if (mode == 2) { // ❌
    u8g2.drawLine(1, 1, 6, 6);
    u8g2.drawLine(6, 1, 1, 6);
  }
  u8g2.sendBuffer();
}
