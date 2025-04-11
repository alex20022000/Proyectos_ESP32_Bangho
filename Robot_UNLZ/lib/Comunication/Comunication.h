#pragma once
#include <Arduino.h>

void initSerial();
void readSerial();
void processCommand(const char* cmd);

// Funciones que por ahora estan implementadas en el main
extern void home();
extern void moveMotor(int motor, int steps);