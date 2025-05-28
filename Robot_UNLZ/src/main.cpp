#include <Arduino.h>
#include <Wire.h>

#define STEP_PIN_Q1 14
#define STEP_PIN_Q2 16
#define STEP_PIN_Q3 19
#define DIR_PIN_Q1 13
#define DIR_PIN_Q2 4
#define DIR_PIN_Q3 18
#define M0_PIN 32
#define M1_PIN 33
#define M2_PIN 25
#define ENDSTOP_SW1_PIN 35
#define ENDSTOP_SW2_PIN 34
#define ENDSTOP_SW3_PIN 36
#define ANTIHORARIO HIGH
#define HORARIO LOW
#define PISADO HIGH
#define NO_PISADO LOW
#define L1 136.65
#define L2 150
#define L3 150
#define Q3_OFFSET 130.0 // Offset de Q3 respecto a la posicion de DH
// Límites de ejes en grados
const float Q1_MIN = 0.0;
const float Q1_MAX = 180.0;
const float Q2_MIN = 0.0;
const float Q2_MAX = 160.0;
const float Q3_MIN = 0.0;
const float Q3_MAX = -300.0;

const float STEP = 10.0; // resolución del muestreo en mm
const float X_MIN = -150, X_MAX = +150;
const float Z_MIN = 0, Z_MAX = 400;
float y = 45; // si trabajas en el plano X-Z

// Variables globales para estados de finales de carrera
volatile bool end_sw1 = false;
volatile bool end_sw2 = false;
volatile bool end_sw3 = false;
const int debounceThreshhold = 100; // Tiempo de debounce en ms

// Constantes de scan del multiplexor
const byte start_address = 8;
const byte end_address = 119;
#define TCAADDR 0x70
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// Define las reducciones por motor
const float reductionQ[4] = {
    1.0f, // índice 0 sin usar
    1.0f, // motor 1: relación 1:1
    4.0f, // motor 2: relación 1:4
    1.0f  // motor 3: relación 1:1
};

// Variable que guarda el salto de angulo actual
float stepAngle = 1.8 / 16; // Valor por defecto (1/16 Step)

// Variable de cant de pasos por revolucion, por defecto para full step
int stepsPerRev = round(360.0 / stepAngle); // Valor por defecto (Full Step)

// Variable para modificar el tiempo entre cambio de flancos (us)
u_int16_t deltaPulseTime = 1500;        // Tiempo default
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
bool usarRampa = true;                                                              // Booleano para activar/desactivar la rampa
int acelFactor = 50;                                                                // Acelfactor determina que porcentaje de la velocidad actual se va a usar para la rampa trapezoidal
unsigned long MIN_SPEED = deltaPulseTime + ((acelFactor / 100.0) * deltaPulseTime); // Tiempo en µs para velocidad mínima (más lento = mayor delay)
unsigned long MAX_SPEED = deltaPulseTime - ((acelFactor / 100.0) * deltaPulseTime); // Tiempo en µs para velocidad máxima (más rápido = menor delay)

// Filtro EMA
float alpha = 0.9; // Valor entre 0 y 1, donde 0 es sin filtrado y 1 es filtrado completo

// Encoders
struct encoderData
{
  float angle;         // Angulo en grados
  float filteredAngle; // Angulo filtrado
  volatile float previousEMAValue = 0;
};

// Encoders - timing de actualizacion
const int ENCODERS_UPDATE_INTERVAL_MS = 10; // Intervalo de actualización en ms
unsigned long lastEncoderReadTime = 0;      // Variable para almacenar el tiempo de la última lectura

encoderData encoders[3]; // Estructura para almacenar los datos de los encoders

// Angulos de cinematica inversa
float angsToMove[3];

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
void capturarEstadoSwitches();
void procesarComandoStep(String command);
void procesarComandoDelta(String command);
void procesarComandoAng(String command);
void procesarComandoRampa(String command);
void procesarComandoMove3(String command);
void procesarComandoMoveCoord(String command);
bool esAlcanzableDesdeHome(float q1, float q2, float q3);
void mostrarMenuMicrostepping();
void mostrarInfo();
float calculateRPM();
IRAM_ATTR void ISR_sw1();
IRAM_ATTR void ISR_sw2();
IRAM_ATTR void ISR_sw3();
void scanI2CBus();
void tcaselect(uint8_t i);
void encodersInfo();
uint8_t CheckEncodersStatus();
uint16_t readEncoder(uint8_t chan_encoder);
void updateAngleFilteredValue(uint8_t chanEncoder);
void move3Motors(float ang1, float ang2, float ang3);
void inverseKinematics(float px, float py, float pz);
void barridoPuntos();

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
  // Finales de carrera
  pinMode(ENDSTOP_SW1_PIN, INPUT); // Solo INPUT ya que tengo resistencias de 10k en pull-down
  pinMode(ENDSTOP_SW2_PIN, INPUT);
  pinMode(ENDSTOP_SW3_PIN, INPUT);
  // Microstepping default config
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  delayMicroseconds(1000);
  setMicrostepping(4); // Step (1/16)
  setDeltaPulseTime(deltaPulseTime);

  // Configuracion de interrupciones para los finales de carrera
  attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW1_PIN), ISR_sw1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW2_PIN), ISR_sw2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW3_PIN), ISR_sw3, CHANGE);
  capturarEstadoSwitches(); // Leo y guardo el estado de los finales de carrera al inicio

  Wire.begin(I2C_SDA, I2C_SCL);
  scanI2CBus(); // Escaneo el bus I2C para ver que dispositivos tengo conectados

  // Inicializo valores para los encoders
  for (uint8_t i = 0; i < 3; i++)
  {
    encoders[i].previousEMAValue = 0.0;
    encoders[i].filteredAngle = 0.0;
  }

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
    else if (command.startsWith("move3"))
    {
      procesarComandoMove3(command);
    }
    else if (command.startsWith("movecoord"))
    {
      procesarComandoMoveCoord(command);
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
      capturarEstadoSwitches();
    }
    else if (command.startsWith("rampa"))
    {
      procesarComandoRampa(command);
    }
    else
    {
      Serial.println("Command unrecognized");
    }
  }

  // Actualizo los encoders cada ENCODERS_UPDATE_INTERVAL_MS
  if (millis() - lastEncoderReadTime >= ENCODERS_UPDATE_INTERVAL_MS)
  {
    lastEncoderReadTime = millis(); // Actualiza el tiempo de la última lectura
    // Actualizo los 3 filtros
    for (uint8_t i = 0; i < 3; i++)
    {
      updateAngleFilteredValue(i);
    }
  }
  // for (int i = 0; i < 1; i++)
  //{
  //   barridoPuntos();
  // }

  /*
  inverseKinematics(-200.0, 0.0, 100.0);
  for (int i = 0; i < 3; i++)
  {
    Serial.println(angsToMove[i]);
  }
  */
  // inverseKinematics(250.0, 250.0, 140.0);
  /*
  for (int i = 0; i < 3; i++)
  {
   Serial.println(angsToMove[i]);
  }
   */
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
  /*while (end_sw1 == NO_PISADO)
  {
    Serial.println("Debuging Q1");
    doAStep(STEP_PIN_Q1, deltaHommingPulseTime);
  }
  while (end_sw2 == NO_PISADO)
  {
    Serial.println("Debuging Q2");
    doAStep(STEP_PIN_Q2, deltaHommingPulseTime);
  }
  while (end_sw3 == NO_PISADO)
  {
    Serial.println("Debuging Q3");
    doAStep(STEP_PIN_Q3, deltaHommingPulseTime);
  }*/

  while (digitalRead(ENDSTOP_SW3_PIN) == NO_PISADO)
  {
    doAStep(STEP_PIN_Q3, deltaHommingPulseTime);
  }
  while (digitalRead(ENDSTOP_SW2_PIN) == NO_PISADO)
  {
    doAStep(STEP_PIN_Q2, deltaHommingPulseTime);
  }
  while (digitalRead(ENDSTOP_SW1_PIN) == NO_PISADO)
  {
    doAStep(STEP_PIN_Q1, deltaHommingPulseTime);
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
  // Escala los pasos según la reducción
  int realSteps = abs(steps) * reductionQ[motor];
  // Mover la cantidad de pasos solicitada
  if (usarRampa)
    moverPasosTrapezoidal(stepPin, dirPin, abs(realSteps));
  else
  {
    moverPasos(stepPin, dirPin, abs(realSteps));
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

// === FUNCION QUE PROCESA EL COMANDO DE CAMBIO DE VELOCIDAD ===
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
// === FUNCION QUE PROCESA EL COMANDO PARA ACTIVAR O DESACTIVAR LA RAMPA ===
void procesarComandoRampa(String command)
{
  // Esperamos "rampa on" o "rampa off"
  if (command.equalsIgnoreCase("rampa on"))
  {
    usarRampa = true;
    Serial.println("✅ Rampa trapezoidal ACTIVADA");
  }
  else if (command.equalsIgnoreCase("rampa off"))
  {
    usarRampa = false;
    Serial.println("✅ Rampa trapezoidal DESACTIVADA");
  }
  else
  {
    Serial.println("❌ Comando inválido. Use: rampa on | rampa off");
  }
}
void procesarComandoMove3(String command)
{
  float a1, a2, a3;
  // Intentamos parsear tres floats tras la palabra "move3"
  int parsed = sscanf(command.c_str(), "move3 %f %f %f", &a1, &a2, &a3);
  if (parsed == 3)
  {
    // Verificar si los ángulos están dentro de los límites

    Serial.print("➡️ Moviendo ejes a: ");
    Serial.print(a1);
    Serial.print("°, ");
    Serial.print(a2);
    Serial.print("°, ");
    Serial.print(a3);
    Serial.println("°");
    move3Motors(a1, a2, a3);
  }
  else
  {
    Serial.println("❌ Comando inválido. Use: move3 <ang1> <ang2> <ang3>");
  }
}
void procesarComandoMoveCoord(String command)
{
  float px, py, pz;
  int parsed = sscanf(command.c_str(), "movecoord %f %f %f", &px, &py, &pz);
  if (parsed == 3)
  {
    inverseKinematics(px, py, pz);
    float q1 = angsToMove[0],
          q2 = angsToMove[1],
          q3 = angsToMove[2];
    Serial.printf("→ IK: (%.1f,%.1f,%.1f) → q1=%.2f°, q2=%.2f°, q3=%.2f°\n",
                  px, py, pz, q1, q2, q3);

    // Verificar si los ángulos están dentro de los límites
    if (esAlcanzableDesdeHome(q1, q2, q3) == true)
    {
      move3Motors(q1, q2, q3);
    }
    else
    {
      Serial.println("❌ Posición fuera de los limites de trabajo.");
    }
  }
  else
  {
    Serial.println("❌ Uso: movecoord <px> <py> <pz>");
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
  encodersInfo();
}
// === CAPTURA EL ESTADO EN EL MOMENTO DE LOS ENDSTOPS ===
void capturarEstadoSwitches()
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

// === FUNCION QUE ESCANEA EL BUS I2C ===
void scanI2CBus()
{
  Serial.println("\n🔍 Escaneo completo I2C con TCA9548A y AS5600:");

  // Inicia I2C (suponiendo Wire.begin ya en setup())

  // Verifica que el multiplexor responde en 0x70
  Serial.print("Probing TCA9548A en 0x70... ");
  Wire.beginTransmission(0x70);
  if (Wire.endTransmission() != 0)
  {
    Serial.println("FALLO. No se detecta TCA9548A en 0x70!");
    return;
  }
  Serial.println("OK (ACK)");

  // Recorre los 8 canales del multiplexor
  for (uint8_t ch = 0; ch < 8; ch++)
  {
    // Selecciona canal ch
    Wire.beginTransmission(0x70);
    Wire.write(1 << ch);
    Wire.endTransmission();
    delay(1); // tiempo de conmutación

    Serial.printf("\n— Canal %u —\n", ch);

    // 1) Scan genérico de direcciones
    bool anyDevice = false;
    for (uint8_t addr = 1; addr < 127; addr++)
    {
      if (addr == 0x70)
        continue;
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0)
      {
        Serial.printf(" • Dispositivo I2C en 0x%02X\n", addr);
        anyDevice = true;
      }
    }
    if (!anyDevice)
      Serial.println(" • Ningún dispositivo genérico detectado.");

    // 2) Prueba AS5600 en 0x36
    const uint8_t ASADDR = 0x36;
    Wire.beginTransmission(ASADDR);
    if (Wire.endTransmission() == 0)
    {
      Serial.println(" ➤ AS5600 detectado, leyendo registros...");
      // Leer STATUS register (0x0B)
      Wire.beginTransmission(ASADDR);
      Wire.write(0x0B);
      int err1 = Wire.endTransmission();
      if (err1 == 0)
      {
        Wire.requestFrom(ASADDR, (uint8_t)1);
        if (Wire.available())
        {
          uint8_t status = Wire.read();
          Serial.printf("    STATUS = 0x%02X\n", status);
        }
        else
        {
          Serial.println("    ERROR: no hay datos de STATUS");
        }
      }
      else
      {
        Serial.printf("    ERROR al leer STATUS: %d\n", err1);
      }
      // Leer RAW angle (0x0E,0x0F)
      Wire.beginTransmission(ASADDR);
      Wire.write(0x0E);
      int err2 = Wire.endTransmission();
      if (err2 == 0)
      {
        Wire.requestFrom(ASADDR, (uint8_t)2);
        if (Wire.available() >= 2)
        {
          uint16_t raw = Wire.read();
          raw = (raw << 8) | Wire.read();
          raw &= 0x0FFF;
          Serial.printf("    Raw angle = 0x%03X (%u)\n", raw, raw);
          float deg = raw * 360.0f / 4096.0f;
          Serial.printf("    Angle = %.2f°\n", deg);
        }
        else
        {
          Serial.println("    ERROR: no hay datos de ángulo");
        }
      }
      else
      {
        Serial.printf("    ERROR al leer ángulo: %d\n", err2);
      }
    }
    else
    {
      Serial.println(" ➤ No responde AS5600 en 0x36");
    }

    delay(500);
  }

  Serial.println("\n✅ Escaneo completo.");
}

void tcaselect(uint8_t i)
{
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  delay(5);
}

// === FUNCION QUE DIAGNOSTICA EL ESTADO DE LOS AS5600 ===
uint8_t CheckEncodersStatus()
{
  Wire.beginTransmission(0x36); // Dirección I2C del AS5600
  Wire.write(0x0B);             // Registro de estado (STATUS register)
  int error = Wire.endTransmission();

  if (error != 0)
  {
    Serial.print("Error en la transmisión I2C: ");
    Serial.println(error);
    return 0; // Retorna 0 en caso de error
  }

  Wire.requestFrom(0x36, 1); // Solicita 1 byte del registro STATUS
  if (Wire.available())
  {
    return Wire.read(); // Retorna el valor del STATUS register
  }
  else
  {
    Serial.println("Error al leer datos del AS5600");
    return 0; // Retorna 0 si no se reciben los datos esperados
  }
}

uint16_t readEncoder()
{
  Wire.beginTransmission(0x36); // Dirección I2C del AS5600
  Wire.write(0x0E);             // Dirección del registro de ángulo (bits 11:8)
  int error = Wire.endTransmission();

  // Verificar si hubo un error en la transmisión
  if (error != 0)
  {
    Serial.print("Error en la transmisión I2C: ");
    Serial.println(error);
    return 0; // Retorna 0 en caso de error
  }

  // Solicitar 2 bytes del registro de ángulo
  Wire.requestFrom(0x36, 2);
  if (Wire.available() != 2)
  {
    Serial.println("Error al leer datos del AS5600");
    return 0; // Retorna 0 si no se reciben los datos esperados
  }

  // Leer los dos bytes del ángulo
  uint16_t angle = Wire.read();       // Lee el primer byte (0x0E)
  angle = (angle << 8) | Wire.read(); // Lee el segundo byte (0x0F) y combina

  return angle & 0x0FFF; // Aplicar máscara para obtener solo los 12 bits del ángulo
}

// Transformar valor ADC a grados
double rawToDeg(uint16_t value)
{
  return (float)value * 360 / 4095;
}

// === RUTINA PARA LEER ENCODER ===
float leerEncoder(uint8_t chan_encoder, bool printResult = false)
{
  uint8_t num_encoder = chan_encoder + 1; // Numero de encoder (1, 2 o 3)
  tcaselect(chan_encoder);                // Selecciona el canal del multiplexor para el encoder 1
  uint16_t rawValue = readEncoder();      // Valor crudo 12 bits
  float angleDegrees = rawToDeg(rawValue);

  uint8_t reduction = reductionQ[num_encoder];

  float realAngle = angleDegrees / reduction; // Aplica la reducción correspondiente

  // Serial.println("Encoder: " + String(num_encoder) + "-> " + String(realAngle) + "°");

  return realAngle; // Retorna el valor en grados afectado por la reducción
}

void encodersInfo()
{
  leerEncoder(0, true); // Encoder 1
  delay(50);
  leerEncoder(1, true); // Encoder 2
  delay(50);
  leerEncoder(2, true); // Encoder 3
  delay(50);
}

void move3Motors(float ang1, float ang2, float ang3)
{
  // 1.Transformo angulos en pasos
  int steps1 = angleToSteps(ang1);
  int steps2 = angleToSteps(ang2);
  int steps3 = angleToSteps(ang3);

  // 2.Afecto por la reducción
  int realSteps1 = abs(steps1) * reductionQ[1]; // Motor 1
  int realSteps2 = abs(steps2) * reductionQ[2]; // Motor 2
  int realSteps3 = abs(steps3) * reductionQ[3]; // Motor 3

  // 3.Seteo las direcciones de los motores segun el signo de los angulos
  bool dir1 = (ang1 > 0) ? ANTIHORARIO : HORARIO;
  bool dir2 = (ang2 > 0) ? ANTIHORARIO : HORARIO;
  bool dir3 = (ang3 > 0) ? ANTIHORARIO : HORARIO;
  digitalWrite(DIR_PIN_Q1, dir1); // Motor 1
  digitalWrite(DIR_PIN_Q2, dir2); // Motor 2
  digitalWrite(DIR_PIN_Q3, dir3); // Motor 3

  // Motor que mas pasos necesita
  int maxSteps = max(realSteps1, max(realSteps2, realSteps3));
  if (maxSteps == 0)
    return;

  // Inicializo errores
  int errA = 0, errB = 0, errC = 0;

  // 4.Bresenham con rampa trapezoidal
  if (usarRampa == true)
  {
    // Fases de la rampa trapezoidal
    int accelSteps = maxSteps / 3;                       // Pasos de aceleración
    int decelSteps = maxSteps / 3;                       // Pasos de desaceleración
    int constSteps = maxSteps - accelSteps - decelSteps; // Pasos constantes

    // Para acelerar, la velocidad va de MIN_SPEED a MAX_SPEED. La fase de aceleracion dura ciertos pasos (totalSteps / 3).
    // Entonces, si tiene que llegar desde MIN_SPEED a MAX_SPEED en tantos pasos, el deltadelay es esa diferencia entre pulsos dividido por la cantidad de pasos de aceleración.
    unsigned long currentDelay = MIN_SPEED;
    unsigned long deltaDelay = (accelSteps > 0)
                                   ? (MIN_SPEED - MAX_SPEED) / accelSteps
                                   : 0;

    // 4️⃣ Inicializo errores Bresenham
    int errA = 0, errB = 0, errC = 0;

    // 5️⃣ Bucle maestro: maxSteps “ticks”
    for (int i = 0; i < maxSteps; i++)
    {
      // 🔄 Actualizo currentDelay según fase
      if (i < accelSteps && currentDelay > MAX_SPEED)
      {
        currentDelay = max(currentDelay - deltaDelay, MAX_SPEED);
      }
      else if (i >= accelSteps + constSteps && currentDelay < MIN_SPEED)
      {
        currentDelay = min(currentDelay + deltaDelay, MIN_SPEED);
      }
      // ⚖️ Acumulo errores
      errA += realSteps1;
      errB += realSteps2;
      errC += realSteps3;

      // 🌀 Disparo pasos cuando el error cruza el umbral
      if (errA >= maxSteps)
      {
        doAStep(STEP_PIN_Q1, currentDelay);
        errA -= maxSteps;
      }
      if (errB >= maxSteps)
      {
        doAStep(STEP_PIN_Q2, currentDelay);
        errB -= maxSteps;
      }
      if (errC >= maxSteps)
      {
        doAStep(STEP_PIN_Q3, currentDelay);
        errC -= maxSteps;
      }
    }

    // 4.Bresenham sin rampa
    if (usarRampa == false)
    {
      for (int i = 0; i < maxSteps; i++)
      {
        // Calculo el error de cada motor
        errA += realSteps1;
        errB += realSteps2;
        errC += realSteps3;
        // Si el error supera el maximo, hago un paso
        if (errA >= maxSteps)
        {
          doAStep(STEP_PIN_Q1, deltaPulseTime);
          errA -= maxSteps; // Reinicio el error
        }

        if (errB >= maxSteps)
        {
          doAStep(STEP_PIN_Q2, deltaPulseTime);
          errB -= maxSteps; // Reinicio el error
        }
        if (errC >= maxSteps)
        {
          doAStep(STEP_PIN_Q3, deltaPulseTime);
          errC -= maxSteps; // Reinicio el error
        }
      }
    }
  }
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

// === FUNCION QUE ACTUALIZA EL VALOR DEL ENCODER Y APLICA FILTRO EMA ===

void updateAngleFilteredValue(uint8_t chanEncoder)
{
  // Leemos el ángulo crudo en grados
  float rawAngle = leerEncoder(chanEncoder);

  // Aplicamos filtro ema
  float ema = alpha * rawAngle + (1.0 - alpha) * encoders[chanEncoder].previousEMAValue;

  // Guardamos el valor filtrado y el valor anterior
  encoders[chanEncoder].previousEMAValue = ema;
  encoders[chanEncoder].filteredAngle = ema;
}
float getAngleFilteredValue(uint8_t nEncoder)
{
  // nEncoder en 1..3 → índice de canal 0..2 del multiplexor
  return encoders[nEncoder - 1].filteredAngle;
}

void inverseKinematics(float px, float py, float pz)
{
  /*
  // 1. Calculo en radianes
  float q1 = atan2(py, px); // [rad]
  // 2. Pasaje a grados
  float q1_deg = q1 * (180.0 / PI); // [°]
  // 3. offset para este caso no hay ya que no participa q3

  // 1. Calculo en radianes
  float q3 = atan2(sqrt(1-(pow(pow(px,2)+pow(py,2)+pow(pz-L1,2)-pow(L2,2)-pow(L3,2)/2*L2*L3,2))), pow(px,2)+pow(py,2)+pow(pz-L1,2)-pow(L2,2)-pow(L3,2)/2*L2*L3);
  // 2. Pasaje a grados
  float q3_deg = q3 * (180.0 / PI); // [°]
  // 3. offset debido a la diferencia entre el DH y nuestro home
  float q3_deg_corrected = q3_deg + Q3_OFFSET; // [°]


  // 1. Calculo en radianes
  float q2 = atan2(pz-L1, sqrt(pow(px, 2) + pow(py, 2) - atan2(L3*sqrt(1-pow(cos(q3), 2)), L2 + (L3*(pow(px,2)+pow(py,2)+pow(pz-L1,2)*pow(L2,2)*pow(L3,2)/2*L2*L3)))));
  // 2. Pasaje a grados
  float q2_deg = q2 * (180.0 / PI); // [°]

  angsToMove[0] = q1_deg;
  angsToMove[1] = q2_deg;
  angsToMove[2] = q3_deg_corrected;
  */

  // Entrada: px, py, pz (mm)
  float q1 = atan2(py, px); // [rad]

  float num = ((pow(px, 2) + pow(py, 2) + (pow((pz - L1), 2))) - pow(L2, 2) - pow(L3, 2));
  float den = (2 * L2 * L3);
  float cos_q3 = num / den;
  cos_q3 = constrain(cos_q3, -1.0, 1.0); // Aseguramos que esté en rango [-1, 1] para evitar NaN
  float q3 = -acos(cos_q3);              // [rad]

  // q2
  float num2 = (pz - L1) / sqrt(pow(px, 2) + pow(py, 2));
  float den2 = (L3 * sin(q3)) / (L2 + (L3 * cos(q3)));
  float q2 = atan(num2) - atan(den2); // [rad]

  // Si quieres grados:
  float q1_deg = q1 * 180.0 / PI;
  float q2_deg = q2 * 180.0 / PI;
  float q3_deg = q3 * 180.0 / PI;

  // Corrigo q3 para el offset
  float q3_deg_corrected = q3_deg - Q3_OFFSET;

  Serial.printf("IK: px=%.2f py=%.2f pz=%.2f → q1=%.2f° q2=%.2f° q3-offset=%.2f° q3=%.2f°\n",
                px, py, pz, q1_deg, q2_deg, q3_deg, q3_deg_corrected);

  // Corrigo el signo de q3 ya que nuestro motor considera el sentido antihorario positivo
  // q3_deg_corrected = q3_deg_corrected * -1.0;

  // Resultado
  angsToMove[0] = q1_deg;
  angsToMove[1] = q2_deg;
  angsToMove[2] = q3_deg_corrected;
}

bool esAlcanzableDesdeHome(float q1, float q2, float q3)
{

  if (q1 >= Q1_MIN && q1 <= Q1_MAX &&
      q2 >= Q2_MIN && q2 <= Q2_MAX &&
      q3 <= Q3_MIN && q3 >= Q3_MAX)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void barridoPuntos()
{
  for (float x = X_MIN; x <= X_MAX; x += STEP)
  {
    for (float z = Z_MIN; z <= Z_MAX; z += STEP)
    {
      inverseKinematics(x, y, z);
      float q1 = angsToMove[0],
            q2 = angsToMove[1],
            q3 = angsToMove[2];

      bool ok = (q1 >= Q1_MIN && q1 <= Q1_MAX) && (q2 >= Q2_MIN && q2 <= Q2_MAX) && (q3 >= Q3_MIN && q3 <= Q3_MAX);

      if (ok)
      {
        // (x,z) es alcanzable dentro de tus restricciones
        Serial.printf("OK: x=%.1f z=%.1f → q1=%.1f q2=%.1f q3=%.1f\n",
                      x, z, q1, q2, q3);
      }
    }
  }
}