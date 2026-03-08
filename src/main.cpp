#include <Arduino.h>

hw_timer_t * timer = NULL;
// Usamos 'volatile' para que el procesador sepa que esta variable cambia en la ISR
volatile bool timerInterrumpido = false; 

void IRAM_ATTR onTimer() {
  timerInterrumpido = true; // Solo levantamos la bandera
}

void setup() {
  // 1. Iniciamos la comunicación serial a 115200 baudios
  Serial.begin(115200); 
  
  pinMode(2, OUTPUT);

  // Configuración del timer (1 segundo)
  timer = timerBegin(1, 80, true);  // timer 1, divisor 80, countUp
  timerAttachInterrupt(timer, &onTimer, true);  // true = edge trigger
  timerAlarmWrite(timer, 1000000, true);  // 1 segundo (1,000,000 µs), autoreload
  timerAlarmEnable(timer);  // habilitar alarma

  Serial.println("Sistema iniciado. Esperando interrupción...");
}

void loop() {
  // 2. Revisamos si la bandera se levantó
  if (timerInterrumpido) {
    timerInterrumpido = false; // Bajamos la bandera
    
    // 3. Ahora sí, imprimimos en consola de forma segura
    Serial.println("¡Interrupción activada! Ha pasado 1 segundo.");
    digitalWrite(2, !digitalRead(2)); // Seguimos parpadeando el LED para confirmar
  }
}