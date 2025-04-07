#include <Arduino.h>
#include <Endstops.h>

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


// Variable que guarda el salto de angulo actual
float stepAngle = 1.8 / 16; // Valor por defecto (1/16 Step)

// Variable de cant de pasos por revolucion, por defecto para full step
int stepsPerRev = round(360.0 / stepAngle); // Valor por defecto (Full Step)

// Variable para modificar el tiempo entre cambio de flancos (us)
u_int16_t deltaPulseTime = 1500; // Tiempo default
u_int16_t deltaHommingPulseTime = 1500; // Tiempo default


// Modos de microstepping
int microsteppingMode = 4; // Modo por defecto
String microsteppingModes[8] = {
    "0 - > Full Step (1/1) -> Default -> 1.8°",
    "1 - > Half Step (1/2) -> " + String(1.8 / 2) + "°",
    "2 - > 1/4 Step -> " + String(1.8 / 4) + "°",
    "3 - > 1/8 Step -> " + String(1.8 / 8) + "°",
    "4 - > 1/16 Step -> " + String(1.8 / 16) + "°",
    "5 - > 1/32 Step -> " + String(1.8 / 32) + "°",
    "6 - > 1/32 Step -> " + String(1.8 / 32) + "°",
    "7 - > 1/32 Step -> " + String(1.8 / 32) + "°"};

// Rampa de aceleracion - configuración de la rampa trapezoidal
bool usarRampa = true; // Booleano para activar/desactivar la rampa
int acelFactor = 50;
unsigned long MIN_SPEED = deltaPulseTime + ((acelFactor / 100.0) * deltaPulseTime); // Tiempo en µs para velocidad mínima (más lento = mayor delay)
unsigned long MAX_SPEED = deltaPulseTime - ((acelFactor / 100.0) * deltaPulseTime); // Tiempo en µs para velocidad máxima (más rápido = menor delay)

// Prototipos
void home();
void doAStep(int stepPin, uint16_t deltaPulseTime);
void moveMotor(int motor, int steps);
void moverPasos(int stepPin, int dirPin, int pasos);
void moverPasosTrapezoidal(int stepPin, int dirPin, int pasos);
int angleToSteps(float angulo);
void setMicrostepping(int mode);
void setStepsPerRev();
void setDeltaPulseTime(unsigned long deltaPulseTime);
float getPulsePeriod();
float getPulseFrecuency();
void procesarComandoStep(String command);
void procesarComandoDelta(String command);
void procesarComandoAng(String command);
void mostrarMenuMicrostepping();
void mostrarInfo();
float calculateRPM();

void setup()
{
  // Configura el puerto serie
  Serial.begin(115200);
  // Configuración de pines
  // Motores
  pinMode(STEP_PIN_Q1, OUTPUT);
  pinMode(STEP_PIN_Q2, OUTPUT);
  pinMode(STEP_PIN_Q3, OUTPUT);
  pinMode(DIR_PIN_Q1, OUTPUT);
  pinMode(DIR_PIN_Q2, OUTPUT);
  pinMode(DIR_PIN_Q3, OUTPUT);

  // Microstepping default config
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  delayMicroseconds(1000);
  setMicrostepping(4); // Step (1/16)
  setDeltaPulseTime(deltaPulseTime);

  captureEndstopStates(); // Leo y guardo el estado de los finales de carrera al inicio
  // Mensaje de bienvenida
  Serial.println("🚀 Ingrese 'home' para iniciar");
}

void loop()
{

  // Verifica si hay datos en el serial
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Elimina los espacios al principio y al final

    // Imprimir el comando recibido para depuración
    Serial.println("Cmd rec -->: " + command);

    if (command == "home")
    {
      // Mover a home
      home();
    }
    else if (command.startsWith("move"))
    {
      // Procesar el comando "move <motor> <pasos>"
      int motor, pasos;
      if (sscanf(command.c_str(), "move %d %d", &motor, &pasos) == 2)
      {
        moveMotor(motor, pasos);
      }
      else
      {
        Serial.println("❌ Comando inválido. Use: move <motor> <pasos>");
      }
    }
    // Procesar el comando "move <motor> <modo>"
    else if (command.startsWith("step"))
    {
      procesarComandoStep(command);
    }
    // Procesar el comando "ang <motor> <angulo>"
    else if (command.startsWith("ang"))
    {
      procesarComandoAng(command);
    }
    // Procesar el comando "delta <valor> [us]"
    else if (command.startsWith("delta"))
    {
      procesarComandoDelta(command);
    }
    else if (command == "info")
    {
      mostrarInfo();
    }
    else if (command == "switch")
    {
      captureEndstopStates();
    }
    else
    {
      Serial.println("Command unrecognized");
    }
  }
}

// === FUNCION PARA REALIZAR EL HOMING ===
void home()
{
  Serial.println("Checking endstops states...");

  // Config direcciones hacia finales de carrera
  digitalWrite(DIR_PIN_Q1, HORARIO);     // Horaria
  digitalWrite(DIR_PIN_Q2, HORARIO);     // Horaria
  digitalWrite(DIR_PIN_Q3, ANTIHORARIO); // Anti-Horaria

  // Verificar si algun final de carrera se encuentra pisado
  bool sw1 = digitalRead(ENDSTOP_SW1_PIN);
  bool sw2 = digitalRead(ENDSTOP_SW2_PIN);
  bool sw3 = digitalRead(ENDSTOP_SW3_PIN);

  if (sw1 || sw2 || sw3)
  {
    // Si alguno de los finales de carrera está pisado, retroceder
    int angleBack = 40; // Grados a retroceder
    setMicrostepping(5);
    int stepsBack = angleToSteps(angleBack); // Pasos a retroceder
    // Retroceso hasta que se liberen los finales de carrera
    if (sw1)
    {
      Serial.println("SW1 Pressed, going back Q1...");
      digitalWrite(DIR_PIN_Q1, ANTIHORARIO); // cambiar direccion para alejarse
      for (int i = 0; i < stepsBack; i++)
      {
        digitalWrite(STEP_PIN_Q1, HIGH);
        delayMicroseconds(deltaHommingPulseTime);
        digitalWrite(STEP_PIN_Q1, LOW);
        delayMicroseconds(deltaHommingPulseTime);
      }
    }
    if (sw2)
    {
      Serial.println("SW2 Pressed, going back Q2...");
      digitalWrite(DIR_PIN_Q2, ANTIHORARIO); // cambiar direccion para alejarse
      for (int i = 0; i < stepsBack; i++)
      {
        digitalWrite(STEP_PIN_Q2, HIGH);
        delayMicroseconds(deltaHommingPulseTime);
        digitalWrite(STEP_PIN_Q2, LOW);
        delayMicroseconds(deltaHommingPulseTime);
      }
    }
    if (sw3)
    {
      Serial.println("SW3 Pressed, going back Q3...");
      digitalWrite(DIR_PIN_Q3, HORARIO); // cambiar direccion para alejarse
      for (int i = 0; i < stepsBack; i++)
      {
        digitalWrite(STEP_PIN_Q3, HIGH);
        delayMicroseconds(deltaHommingPulseTime);
        digitalWrite(STEP_PIN_Q3, LOW);
        delayMicroseconds(deltaHommingPulseTime);
      }
    }
  }
  Serial.println("Checking endstops OK...");
  // A partir de aca, iniciar la rutina de homing
  // Setear direccion hacia los finales de carrera
  digitalWrite(DIR_PIN_Q1, HORARIO);     // Horaria
  digitalWrite(DIR_PIN_Q2, HORARIO);     // Horaria
  digitalWrite(DIR_PIN_Q3, ANTIHORARIO); // Anti-Horaria

  // Mover hasta que se presione el final de carrera
  while (end_sw1 == NO_PISADO)
  {
    doAStep(STEP_PIN_Q1, deltaHommingPulseTime);
  }
  while (end_sw2 == NO_PISADO)
  {
    doAStep(STEP_PIN_Q2, deltaHommingPulseTime);
  }
  while (end_sw3 == NO_PISADO)
  {
    doAStep(STEP_PIN_Q3, deltaHommingPulseTime);
  }

  // homming completado
  Serial.println("🏠✅ Homing completado.");
}

// === FUNCION QUE REALIZA UN PASO ===
void doAStep(int stepPin, uint16_t deltaPulseTime = deltaPulseTime)
{
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(deltaPulseTime);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(deltaPulseTime);
}

// === FUNCION PARA MOVER UN MOTOR POR CANT DE PASOS ===
void moveMotor(int motor, int steps)
{
  int stepPin, dirPin;

  // Asignar pines según el motor seleccionado
  switch (motor)
  {
  case 1:
    stepPin = STEP_PIN_Q1;
    dirPin = DIR_PIN_Q1;
    break;
  case 2:
    stepPin = STEP_PIN_Q2;
    dirPin = DIR_PIN_Q2;
    break;
  case 3:
    stepPin = STEP_PIN_Q3;
    dirPin = DIR_PIN_Q3;
    break;
  default:
    Serial.println("❌ Número de motor inválido. Use 1, 2 o 3.");
    return;
  }

  // Definir dirección del movimiento
  digitalWrite(dirPin, (steps > 0) ? ANTIHORARIO : HORARIO); // High para horario, Low para antihorario

  // Mover la cantidad de pasos solicitada
  if (usarRampa)
    moverPasosTrapezoidal(stepPin, dirPin, abs(steps));
  else
  {
    moverPasos(stepPin, dirPin, abs(steps));
  }
}

// === FUNCION PARA MOVER UN NUMERO DE PASOS ===
void moverPasos(int stepPin, int dirPin, int pasos)
{
  for (int i = 0; i < pasos; i++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(deltaPulseTime);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(deltaPulseTime);
  }
  Serial.println("✅ Movimiento completado.");
}

// === FUNCION MUEVE UN MOTOR UNA CANT DE PASOS CON RAMPA DE ACEL TRAPEZOIDAL ===
void moverPasosTrapezoidal(int stepPin, int dirPin, int pasos)
{
  int totalSteps = pasos;
  int accelSteps = totalSteps / 3;
  int decelSteps = totalSteps / 3;
  int constSteps = totalSteps - accelSteps - decelSteps;
  unsigned long currentDelay = MIN_SPEED; // Comenzamos con la velocidad más lenta
  unsigned long deltaDelay = (accelSteps > 0) ? (MIN_SPEED - MAX_SPEED) / accelSteps : 0;

  for (int i = 0; i < totalSteps; i++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(currentDelay);

    // Fase de aceleración
    if (i < accelSteps && currentDelay > MAX_SPEED)
    {
      currentDelay = currentDelay - deltaDelay;
      if (currentDelay < MAX_SPEED)
        currentDelay = MAX_SPEED;
    }
    // Fase constante: no se modifica la velocidad (se mantiene en MAX_SPEED)
    // Fase de desaceleración
    else if (i >= (accelSteps + constSteps) && currentDelay < MIN_SPEED)
    {
      currentDelay = currentDelay + deltaDelay;
      if (currentDelay > MIN_SPEED)
        currentDelay = MIN_SPEED;
    }
  }
  Serial.println("✅ Movimiento completado con rampa trapezoidal.");
}

// === FUNCION PARA CAMBIAR EL MODO DE MICROSTEPPING ===
void setMicrostepping(int mode)
{
  const bool modes[8][3] = {
      {LOW, LOW, LOW},   // Paso completo (1/1)
      {HIGH, LOW, LOW},  // Medio paso (1/2)
      {LOW, HIGH, LOW},  // 1/4 de paso
      {HIGH, HIGH, LOW}, // 1/8 de paso
      {LOW, LOW, HIGH},  // 1/16 de paso
      {HIGH, LOW, HIGH}, // 1/32 de paso
      {LOW, HIGH, HIGH}, // 1/32 de paso
      {HIGH, HIGH, HIGH} // 1/32 de paso
  };

  const float stepAngles[8] = {1.8, 1.8 / 2, 1.8 / 4, 1.8 / 8, 1.8 / 16, 1.8 / 32, 1.8 / 32, 1.8 / 32};

  if (mode < 0 || mode > 7)
  {
    Serial.println("❌ Modo inválido. Use un valor entre 0 y 7.");
    return;
  }

  // Configurar los pines M0, M1 y M2
  digitalWrite(M0_PIN, modes[mode][0]);
  digitalWrite(M1_PIN, modes[mode][1]);
  digitalWrite(M2_PIN, modes[mode][2]);

  // Actualizar el ángulo por paso
  stepAngle = stepAngles[mode];

  // Actualizar la cantidad de pasos por revolución
  setStepsPerRev();

  // Actualizar el modo actual en la variable global
  microsteppingMode = mode;

  Serial.print("✅ Microstepping ajustado a: ");
  Serial.println(microsteppingModes[mode]);
  Serial.print("📏 Paso angular actual: ");
  Serial.print(stepAngle);
  Serial.println("°");
}

// === FUNCION PARA PROCESAR EL COMANDO <STEP> ===
void procesarComandoStep(String command)
{
  if (command == "step")
  {
    mostrarMenuMicrostepping();
  }
  else
  {
    int mode;
    if (sscanf(command.c_str(), "step %d", &mode) == 1)
    {
      setMicrostepping(mode);
    }
    else
    {
      Serial.println("❌ Comando inválido. Use: step <modo> (0 a 5)");
    }
  }
}

// === FUNCION QUE MUESTRA EL MENU DE MODIFICACION DE MICROSTEPPING ===
void mostrarMenuMicrostepping()
{
  Serial.println("\n🔧 Seleccione la resolución deseada:");
  Serial.println("0. Paso completo [1/1 ; 1.8°]");
  Serial.println("1. Medio paso [1/2 ; 0.9°]");
  Serial.println("2. 1/4 de paso [1/4 ; 0.45°]");
  Serial.println("3. 1/8 de paso [1/8 ; 0.225°]");
  Serial.println("4. 1/16 de paso [1/16 ; 0.1125°]");
  Serial.println("5. 1/32 de paso [1/32 ; 0.05625°]");
  Serial.println("6. 1/32 de paso [1/32 ; 0.05625°]");
  Serial.println("7. 1/32 de paso [1/32 ; 0.05625°]");
  Serial.println("👉 Ingrese el número de la opción:");
}

// === FUNCION QUE PROCESA EL COMANDO DE CAMVIO DE VELOCIDAD ===
void procesarComandoDelta(String command)
{
  long value;
  if (sscanf(command.c_str(), "delta %ld", &value) == 1)
  {
    setDeltaPulseTime(value);
  }
  else
  {
    Serial.println("❌ Comando inválido. Use: delta <valor>");
  }
}

// === FUNCION QUE SETEA LA VELOCIDAD (TIEMPO ENTRE CAMBIO DE FLANCOS) ===
void setDeltaPulseTime(unsigned long value)
{
  deltaPulseTime = value;
  Serial.print("✅ Tiempo entre flancos ajustado a: ");
  Serial.print(deltaPulseTime);
  Serial.println("µs");
}

// === FUNCION QUE PROCESA EL COMANDO PARA MOVER POR ANGULO ===
void procesarComandoAng(String command)
{
  int motor;
  float grados;
  if (sscanf(command.c_str(), "ang %d %f", &motor, &grados) == 2)
  {
    int pasos = angleToSteps(grados);
    Serial.print("Moviendo motor ");
    Serial.print(motor);
    Serial.print(" ");
    Serial.print(grados);
    Serial.print("° (");
    Serial.print(pasos);
    Serial.println(" pasos)");
    moveMotor(motor, pasos);
  }
  else
  {
    Serial.println("❌ Comando inválido. Use: ang <motor> <grados>");
  }
}

// === FUNCION QUE TRANSFORMA DE ANGULO A PASOS ===
int angleToSteps(float angulo)
{
  return (int)round(angulo / stepAngle);
}

// === FUNCION QUE CALCULA LA CANT DE PASOS POR REV DEPENDIENDO EL MICROSTEPPING SELECCIONADO ===
void setStepsPerRev()
{
  stepsPerRev = (int)round(360.0 / stepAngle);
}

// === FUNCION QUE CALCULA LA VELOCIDAD EN RPM ===
float calculateRPM()
{
  float RPM = (getPulseFrecuency() * 60) / stepsPerRev;
  return RPM;
}

// === FUNCION QUE CALCULA EL PERIODO DEL PULSO STEP ===
float getPulsePeriod()
{
  return deltaPulseTime * 2 / 1000000.0; // de us a segundos
}

// === FUNCION QUE CALCULA LA FRECUENCIA DEL PULSO STEP ===
float getPulseFrecuency()
{
  return 1 / getPulsePeriod(); // en Hz
}

// === FUNCION QUE MUESTRA INFO SOBRE VARIABLES DE INTERES ===
void mostrarInfo()
{
  Serial.println("\n====  Información Actual ====");
  Serial.print("Modo de Microstepping: ");
  Serial.println(microsteppingModes[microsteppingMode]);
  Serial.print("Paso angular: ");
  Serial.print(stepAngle);
  Serial.println("°");
  Serial.print("Pasos por revolución: ");
  Serial.println(stepsPerRev);
  Serial.print("ΔT pulso: ");
  Serial.print(deltaPulseTime);
  Serial.println(" µs");
  Serial.print("Período del pulso: ");
  Serial.print(getPulsePeriod(), 6);
  Serial.println(" s");
  Serial.print("Frecuencia del pulso: ");
  Serial.print(getPulseFrecuency(), 6);
  Serial.println(" Hz");
  Serial.print("Velocidad: ");
  Serial.print(calculateRPM(), 6);
  Serial.println(" RPM");
  Serial.println("============================\n");
}


