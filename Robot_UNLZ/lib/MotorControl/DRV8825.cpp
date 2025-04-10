#include <DRV8825.h>

// Contructor y paso de variables privadas
DRV8825::DRV8825(uint8_t _stepPin, uint8_t _dirPin,
                 uint8_t _m0Pin, uint8_t _m1Pin, uint8_t _m2Pin,
                 float _reduction, float _anglePerStep, uint8_t _microsteppingFactor)
    : stepPin(_stepPin),
      dirPin(_dirPin),
      m0Pin(_m0Pin),
      m1Pin(_m1Pin),
      m2Pin(_m2Pin),
      reduction(_reduction),
      anglePerStep(_anglePerStep),
      microsteppingFactor(_microsteppingFactor)
{
    // Inicializa los pines y condiciones iniciales
    init();
}

void DRV8825::init()
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    setMicrosteppingFactor(16);
    setReduction(reduction);
    //setDeltaPulseTime(1000);
}

void DRV8825::setReduction(float _reduction)
{
    reduction = _reduction;
}
float DRV8825::getReduction()
{
    return reduction;
}
void DRV8825::setAnglePerStep()
{
    anglePerStep = anglePerStep/microsteppingFactor;
    setStepsPerRev();
}
void DRV8825::setStepsPerRev(){
    stepsPerRev = (int)round(360 / anglePerStep);
}
float DRV8825::getAnglePerStep()
{
    return anglePerStep;
}
void DRV8825::setMicrosteppingFactor(uint8_t _microsteppingFactor)
{
    microsteppingFactor = _microsteppingFactor;
    // Paso 1: mapear factor a índice
    int index;
    switch (microsteppingFactor)
    {
    case 1:
        index = 0;
        break; // 1/1
    case 2:
        index = 1;
        break; // 1/2
    case 4:
        index = 2;
        break; // 1/4
    case 8:
        index = 3;
        break; // 1/8
    case 16:
        index = 4;
        break; // 1/16
    case 32:
        index = 5;
        break; // 1/32 (usamos sólo una de las tres entradas 1/32)
    default:
        Serial.println("❌ Factor inválido. Use: 1, 2, 4, 8, 16 o 32.");
        return;
    }

    // Paso 2: tabla de estados para M0, M1, M2
    const bool modes[6][3] = {
        {LOW, LOW, LOW},   // 1
        {HIGH, LOW, LOW},  // 2
        {LOW, HIGH, LOW},  // 4
        {HIGH, HIGH, LOW}, // 8
        {LOW, LOW, HIGH},  // 16
        {HIGH, LOW, HIGH}};  // 32

    // Paso 3: configurar pines
    digitalWrite(m0Pin, modes[index][0]);
    digitalWrite(m1Pin, modes[index][1]);
    digitalWrite(m2Pin, modes[index][2]);

    // Paso 4: actualizar el ángulo por paso
    setAnglePerStep();
};
