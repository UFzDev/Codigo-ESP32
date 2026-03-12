#include <Arduino.h>

const int pinBoton = 14; // Pin donde conectaremos el botón

// Variable compartida entre la interrupción y el loop
// 'volatile' asegura que el valor se lea correctamente desde la RAM
volatile bool pulsacionDetectada = false;

//Contador
volatile int contador = 0;

// Función que se ejecuta en la RAM interna
void IRAM_ATTR isrBoton() {
  pulsacionDetectada = true;
  contador++;
}

void setup() {
  // Iniciamos la comunicación serial para mostrar mensajes
  Serial.begin(115200);
  
  // Configuramos el pin con resistencia interna de pull-up 
  pinMode(pinBoton, INPUT_PULLUP);
  
  // attachInterrupt vincula el pin a la función 'isrBoton'
  // El trigger FALLING detecta al presionar
  attachInterrupt(digitalPinToInterrupt(pinBoton), isrBoton, FALLING);

  Serial.println("Sistema listo");
}

void loop() {
  // Solo entramos aquí si el hardware detectó el cambio de voltaje
  if (pulsacionDetectada) {
    pulsacionDetectada = false; // Bajamos la bandera
    
    Serial.println("Boton presionado" + String(contador));
  }
}