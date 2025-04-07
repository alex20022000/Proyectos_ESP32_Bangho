#include "Endstops.h"

// Definición de las variables globales
volatile bool end_sw1 = false;
volatile bool end_sw2 = false;
volatile bool end_sw3 = false;

// === INICIALIZACION DE PINES Y VINCULO DE INTERRUPCIONES DE HW ===
void initEndstops()
{
    // Configuracion de pines de entrada
    pinMode(ENDSTOP_SW1_PIN, INPUT); // Solo INPUT ya que tengo resistencias de 10k en pull-down
    pinMode(ENDSTOP_SW2_PIN, INPUT);
    pinMode(ENDSTOP_SW3_PIN, INPUT);

    // Asociar interrupciones a pines con cambio de flanco
    attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW1_PIN), ISR_sw1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW2_PIN), ISR_sw2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW3_PIN), ISR_sw3, CHANGE);
}

// === CAPTURA EL ESTADO EN EL MOMENTO DE LOS ENDSTOPS ===
void captureEndstopStates()
{
    end_sw1 = digitalRead(ENDSTOP_SW1_PIN);
    end_sw2 = digitalRead(ENDSTOP_SW2_PIN);
    end_sw3 = digitalRead(ENDSTOP_SW3_PIN);
    Serial.print("Estado SW1: ");
    Serial.println(end_sw1 ? "PISADO" : "NO PISADO");
    Serial.print("Estado SW2: ");
    Serial.println(end_sw2 ? "PISADO" : "NO PISADO");
    Serial.print("Estado SW3: ");
    Serial.println(end_sw3 ? "PISADO" : "NO PISADO");
}

// === INTERRUPCIONES ENDSTOPS ===
IRAM_ATTR void ISR_sw1()
{
    static unsigned long timeStampSW1; // Static no destruye el valor al finalizar la funcion

    if (millis() - timeStampSW1 >= debounceThreshhold)
    {
        end_sw1 = !end_sw1;      // Cambia el estado del final de carrera
        timeStampSW1 = millis(); // Actualiza el timestamp
    }
}
IRAM_ATTR void ISR_sw2()
{
    static unsigned long timeStampSW2; // Static no destruye el valor al finalizar la funcion

    if (millis() - timeStampSW2 >= debounceThreshhold)
    {
        end_sw2 = !end_sw2;      // Cambia el estado del final de carrera
        timeStampSW2 = millis(); // Actualiza el timestamp
    }
}
IRAM_ATTR void ISR_sw3()
{
    static unsigned long timeStampSW3; // Static no destruye el valor al finalizar la funcion

    if (millis() - timeStampSW3 >= debounceThreshhold)
    {
        end_sw3 = !end_sw3;      // Cambia el estado del final de carrera
        timeStampSW3 = millis(); // Actualiza el timestamp
    }
}