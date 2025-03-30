#include <Arduino.h>

const int dirPin = 18;
const int stepPin = 4;

const int steps = 200*32;
int stepDelay = 250;

void setup() {
  // Marcar los pines como salida
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
}

void loop() {
  //Activar una direccion y fijar la velocidad con stepDelay
  int stepDelay = 250;
  digitalWrite(dirPin, HIGH);
  // Giramos 200 pulsos para hacer una vuelta completa
  for (int x = 0; x < steps * 1; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
  delay(1000);

  //Cambiamos la direccion y aumentamos la velocidad
  digitalWrite(dirPin, LOW);
  stepDelay = 50;
  // Giramos 400 pulsos para hacer dos vueltas completas
  for (int x = 0; x < steps * 2; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
  delay(1000);
}
