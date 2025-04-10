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
    public:
        DRV8825 (u_int8_t _stepPin, u_int8_t _dirPin, u_int8_t _m0Pin, u_int8_t _m1Pin, u_int8_t _m2Pin, 
                float _reduction=1.0f, float_t _anglePerStep = DEFAULT_ANGLE_PER_STEP, u_int8_t _microsteppingFactor = 1);
    private:
        void init();  
        void doAStep(uint16_t deltaPulseTime = 1500);  
        void setReduction(float _reduction);
        float getReduction();
        void setAnglePerStep();
        float getAnglePerStep();
        void setMicrosteppingFactor(u_int8_t _microsteppingFactor);
        u_int8_t getMicrosteppingFactor();
        void setStepsPerRev();
        int getStepsPerRev();
        //void setDeltaPulseTime(u_int16_t deltaPulseTime);
        //uint16_t getDeltaPulseTime();

        uint8_t stepPin; u_int8_t dirPin; u_int8_t m0Pin; u_int8_t m1Pin; u_int8_t m2Pin;
        float reduction; float anglePerStep; u_int8_t microsteppingFactor;uint8_t microsteppingMode;
        int stepsPerRev;

    };