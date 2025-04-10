#pragma once

#include <Arduino.h>

#define STEP_PIN_Q1 14
#define STEP_PIN_Q2 16
#define STEP_PIN_Q3 19
#define DIR_PIN_Q1 13
#define DIR_PIN_Q2 4
#define DIR_PIN_Q3 18
#define M0_PIN 32
#define M1_PIN 33
#define M2_PIN 25
#define ANTIHORARIO HIGH
#define HORARIO LOW
#define iQ1 1   // relacion de reduccion 1:1
#define iQ2 4   // relacion de reduccion 1:4
#define iQ3 1   // relacion de reduccion 1:1
#define DEFAULT_ANGLE_PER_STEP 1.8 // Grados por paso

class DRV8825
{

};