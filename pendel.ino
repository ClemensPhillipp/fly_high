#include <Servo.h>
#include <math.h>
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"

#define TFT_DC 7
#define TFT_CS 10
#define TFT_RST 8

Adafruit_GC9A01A tft = Adafruit_GC9A01A(TFT_CS, TFT_DC, TFT_RST);

Servo myServo;
Servo s1;
Servo s2;

int IN1_PIN = 2;
int IN2_PIN = 3;
int default_position1 = 110;
int pos1 = default_position1;
int pos2 = default_position1;
int pos = 80;                             // Ausgangsposition (90 Grad)
int timeStep = 10;                        // Zeitschritt in Millisekunden
float t = 0;                              // Initiale Zeit
float g = 9.81;                           // Erdbeschleunigung in m/s^2
float L = 2;                              // Länge des Pendels in Metern (anpassen)
float omega;                              // Winkelgeschwindigkeit
float maxAmplitude = 90;                  // Maximale Amplitude in Grad
float amplitude = 0;                      // Anfangsamplitude
bool increasing = true;                   // Flag für die Amplitudenerhöhung
bool holdMaxAmplitude = false;            // Flag, um anzuzeigen, ob die Amplitude 10 Sekunden lang konstant gehalten wird
unsigned long maxAmplitudeStartTime = 0;  // Zeitpunkt, zu dem die maximale Amplitude erreicht wurde
bool simulationComplete = true;           // Flag, das angibt, ob die Simulation abgeschlossen ist
int rounds = 0;
int x1 = 70;
int x2 = 170;
int y = 80;

void setup() {
  Serial.begin(9600);
  myServo.attach(4);    // Verbinde den Servo mit Pin 12
  myServo.write(pos);   // Setze den Servo auf die Ausgangsposition (90 Grad)
  omega = sqrt(g / L);  // Berechne die Winkelgeschwindigkeit

  s1.attach(5);
  s2.attach(6);
  s1.write(default_position1);
  s2.write(default_position1);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  tft.begin();
}

void loop() {
  if (simulationComplete) {
    rounds = 0;
    showTriangles();
    delay(500);
    showText();
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    openFloor();
    delay(10000);
    closeFloor();
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    delay(7000);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    delay(3000);
    // Setze die Variablen zurück, um eine neue Simulation zu starten
    amplitude = 0;
    t = 0;
    increasing = true;
    holdMaxAmplitude = false;
    simulationComplete = false;
    maxAmplitudeStartTime = 0;
  } else {
    if (rounds == 1) {
      showSmiley();
    }
    // Berechne die neue Position basierend auf der Pendelgleichung
    pos = 80 + amplitude * cos(omega * t);  // Berechne die aktuelle Position des Pendels
    myServo.write(pos);                     // Bewege den Servo auf die neue Position

    // Warte für den nächsten Zeitschritt
    delay(timeStep);

    // Erhöhe die Zeit für den nächsten Schritt
    t += (float)timeStep / 1000;  // Zeit in Sekunden

    // Ändere die Amplitude der Schwingung
    if (increasing) {
      digitalWrite(IN1_PIN, LOW);
      digitalWrite(IN2_PIN, HIGH);
      amplitude += 0.1;  // Langsames Erhöhen der Amplitude
      if (amplitude >= maxAmplitude) {
        amplitude = maxAmplitude;          // Begrenze die Amplitude auf den Maximalwert
        increasing = false;                // Stoppe die Erhöhung der Amplitude
        holdMaxAmplitude = true;           // Beginne die Haltezeit bei maximaler Amplitude
        maxAmplitudeStartTime = millis();  // Speichere den Startzeitpunkt der Haltezeit
      }
    } else if (holdMaxAmplitude) {
      // Überprüfen, ob 10 Sekunden vergangen sind
      if (millis() - maxAmplitudeStartTime >= 30000) {
        holdMaxAmplitude = false;  // Beende die Haltezeit nach 10 Sekunden
      }
    } else {
      digitalWrite(IN1_PIN, LOW);
      digitalWrite(IN2_PIN, LOW);
      amplitude -= 0.1;  // Langsames Verringern der Amplitude
      if (amplitude <= 0) {
        amplitude = 0;              // Begrenze die Amplitude auf 0
        simulationComplete = true;  // Markiere die Simulation als abgeschlossen
      }
    }
    // Überprüfen, ob das Pendel einen vollen Zyklus (hin und zurück) durchlaufen hat
    if (t >= (2 * M_PI / omega)) {
      t = 0;  // Setze die Zeit zurück
    }
  }
  rounds++;
}

void closeFloor() {
  for (pos1; pos1 <= default_position1 + 12; pos1 += 1) {
    s1.write(pos1);
    delay(20);
  }

  for (pos2; pos2 <= default_position1 + 12; pos2 += 1) {
    s2.write(pos2);
    delay(20);
  }
}

void openFloor() {
  for (pos2; pos2 >= default_position1; pos2 -= 1) {
    s2.write(pos2);
    delay(20);
  }

  for (pos1; pos1 >= default_position1; pos1 -= 1) {
    s1.write(pos1);
    delay(20);
  }
}

unsigned long showTriangles() {
  unsigned long start;
  int n, i, cx = tft.width() / 2 - 1,
            cy = tft.height() / 2 - 1;

  tft.fillScreen(GC9A01A_BLACK);
  n = min(cx, cy);
  start = micros();
  for (i = 0; i < n; i += 5) {
    tft.drawTriangle(
      cx, cy - i,      // peak
      cx - i, cy + i,  // bottom left
      cx + i, cy + i,  // bottom right
      tft.color565(i, i, i));
  }

  return micros() - start;
}

unsigned long showText() {
  unsigned long start = micros();
  tft.setCursor(90, 50);
  tft.setTextColor(GC9A01A_WHITE);
  tft.setTextSize(3);
  tft.println("FLY");
  tft.setCursor(60, 100);
  tft.setTextColor(GC9A01A_WHITE);
  tft.setTextSize(5);
  tft.println("HIGH");
  return micros() - start;
}

void showSmiley() {

  tft.fillScreen(GC9A01A_YELLOW);
  //mouth
  tft.fillCircle(120, 120, 80, GC9A01A_RED);
  tft.fillCircle(120, 110, 83, GC9A01A_YELLOW);

  tft.fillCircle(x1, y, 40, GC9A01A_WHITE);
  tft.fillCircle(x1, y, 10, GC9A01A_BLACK);
  tft.fillCircle(x2, y, 40, GC9A01A_WHITE);
  tft.fillCircle(x2, y, 10, GC9A01A_BLACK);
}
