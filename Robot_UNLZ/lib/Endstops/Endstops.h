#pragma once

#include <Arduino.h>

// Definiciones de pines para los endstops
#define ENDSTOP_SW1_PIN 35
#define ENDSTOP_SW2_PIN 34
#define ENDSTOP_SW3_PIN 36

// Definiciones de estados (modificable segun cableado)
#define PISADO HIGH
#define NO_PISADO LOW

// Variables globales para almacenar el estado de los finales de carrera
extern volatile bool end_sw1; // extern ya que se definen en el .cpp
extern volatile bool end_sw2;
extern volatile bool end_sw3;

// Umbral threshold para el debounce (en milisegundos)
const int debounceThreshhold = 100; // Tiempo de debounce en ms

// Prototipos de funciones publicas
void initEndstops(); // Configura los pines e inicializa interrupciones
void captureEndstopStates(); // Captura y muestra el estado de los finales de carrera

// Prototipos de las ISR
IRAM_ATTR void ISR_sw1();
IRAM_ATTR void ISR_sw2();
IRAM_ATTR void ISR_sw3();
