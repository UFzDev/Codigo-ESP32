#include <Arduino.h>

const int pinBoton = 19; // Pin donde conectaremos el botón (D19)
const int pinLed = 23; // Pin del LED verde

void setup() {
  // Configuramos el LED como salida
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, LOW);
  
  // Configuramos el pin con resistencia interna de pull-up 
  pinMode(pinBoton, INPUT_PULLUP);
}

void loop() {
  // Con INPUT_PULLUP: LOW = botón presionado, HIGH = botón suelto
  if (digitalRead(pinBoton) == LOW) {
    digitalWrite(pinLed, HIGH);
  } else {
    digitalWrite(pinLed, LOW);
  }
}