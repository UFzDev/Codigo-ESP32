#include <Arduino.h>

const int pinBoton = 19; // Pin donde conectaremos el botón (D19)
const unsigned long debounceUs = 50000; // 50 ms

// Variable compartida entre la interrupción y el loop
// 'volatile' asegura que el valor se lea correctamente desde la RAM
volatile bool pulsacionDetectada = false;
volatile int contador = 0;
volatile unsigned long ultimaInterrupcionUs = 0;

// Función que se ejecuta en la RAM interna
void IRAM_ATTR isrBoton() {
  unsigned long ahoraUs = micros();
  if (ahoraUs - ultimaInterrupcionUs < debounceUs) {
    return;
  }
  ultimaInterrupcionUs = ahoraUs;

  pulsacionDetectada = true;
  contador++;
}

void setup() {
  // Iniciamos la comunicación serial para mostrar mensajes
  Serial.begin(115200);
  delay(500); // Esperar a que Serial esté listo
  
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

    noInterrupts(); // Deshabilitamos interrupciones para leer el contador de forma segura
    int valorContador = contador;
    contador = 0; // Reiniciar contador a 0
    interrupts(); // Volvemos a habilitar interrupciones

    Serial.println("Boton presionado: " + String(valorContador));
  }
}