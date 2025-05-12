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
  1.0f,  // índice 0 sin usar
  1.0f,  // motor 1: relación 1:1
  4.0f,  // motor 2: relación 1:4
  1.0f   // motor 3: relación 1:1
};

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
void capturarEstadoSwitches();
void procesarComandoStep(String command);
void procesarComandoDelta(String command);
void procesarComandoAng(String command);
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
  // Configuración de interrupciones para finales de carrera
  // Configuracion de interrupciones para los finales de carrera
  attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW1_PIN), ISR_sw1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW2_PIN), ISR_sw2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW3_PIN), ISR_sw3, CHANGE);
  capturarEstadoSwitches(); // Leo y guardo el estado de los finales de carrera al inicio
  
  Wire.begin(I2C_SDA, I2C_SCL);
  scanI2CBus() ; // Escaneo el bus I2C para ver que dispositivos tengo conectados
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
      capturarEstadoSwitches();
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
void scanI2CBus() {
  Serial.println("\n🔍 Escaneo completo I2C con TCA9548A y AS5600:");

  // Inicia I2C (suponiendo Wire.begin ya en setup())

  // Verifica que el multiplexor responde en 0x70
  Serial.print("Probing TCA9548A en 0x70... ");
  Wire.beginTransmission(0x70);
  if (Wire.endTransmission() != 0) {
    Serial.println("FALLO. No se detecta TCA9548A en 0x70!");
    return;
  }
  Serial.println("OK (ACK)");

  // Recorre los 8 canales del multiplexor
  for (uint8_t ch = 0; ch < 8; ch++) {
    // Selecciona canal ch
    Wire.beginTransmission(0x70);
    Wire.write(1 << ch);
    Wire.endTransmission();
    delay(1); // tiempo de conmutación

    Serial.printf("\n— Canal %u —\n", ch);

    // 1) Scan genérico de direcciones
    bool anyDevice = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
      if (addr == 0x70) continue;
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.printf(" • Dispositivo I2C en 0x%02X\n", addr);
        anyDevice = true;
      }
    }
    if (!anyDevice) Serial.println(" • Ningún dispositivo genérico detectado.");

    // 2) Prueba AS5600 en 0x36
    const uint8_t ASADDR = 0x36;
    Wire.beginTransmission(ASADDR);
    if (Wire.endTransmission() == 0) {
      Serial.println(" ➤ AS5600 detectado, leyendo registros...");
      // Leer STATUS register (0x0B)
      Wire.beginTransmission(ASADDR);
      Wire.write(0x0B);
      int err1 = Wire.endTransmission();
      if (err1 == 0) {
        Wire.requestFrom(ASADDR, (uint8_t)1);
        if (Wire.available()) {
          uint8_t status = Wire.read();
          Serial.printf("    STATUS = 0x%02X\n", status);
        } else {
          Serial.println("    ERROR: no hay datos de STATUS");
        }
      } else {
        Serial.printf("    ERROR al leer STATUS: %d\n", err1);
      }
      // Leer RAW angle (0x0E,0x0F)
      Wire.beginTransmission(ASADDR);
      Wire.write(0x0E);
      int err2 = Wire.endTransmission();
      if (err2 == 0) {
        Wire.requestFrom(ASADDR, (uint8_t)2);
        if (Wire.available() >= 2) {
          uint16_t raw = Wire.read();
          raw = (raw << 8) | Wire.read();
          raw &= 0x0FFF;
          Serial.printf("    Raw angle = 0x%03X (%u)\n", raw, raw);
          float deg = raw * 360.0f / 4096.0f;
          Serial.printf("    Angle = %.2f°\n", deg);
        } else {
          Serial.println("    ERROR: no hay datos de ángulo");
        }
      } else {
        Serial.printf("    ERROR al leer ángulo: %d\n", err2);
      }
    } else {
      Serial.println(" ➤ No responde AS5600 en 0x36");
    }

    delay(500);
  }

  Serial.println("\n✅ Escaneo completo.");
}


void tcaselect(uint8_t i) {
  if (i > 7) return;

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

    if (error != 0) {
        Serial.print("Error en la transmisión I2C: ");
        Serial.println(error);
        return 0; // Retorna 0 en caso de error
    }

    Wire.requestFrom(0x36, 1);    // Solicita 1 byte del registro STATUS
    if (Wire.available()) {
        return Wire.read();       // Retorna el valor del STATUS register
    } else {
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
    if (error != 0) {
        Serial.print("Error en la transmisión I2C: ");
        Serial.println(error);
        return 0; // Retorna 0 en caso de error
    }

    // Solicitar 2 bytes del registro de ángulo
    Wire.requestFrom(0x36, 2); 
    if (Wire.available() != 2) {
        Serial.println("Error al leer datos del AS5600");
        return 0; // Retorna 0 si no se reciben los datos esperados
    }

    // Leer los dos bytes del ángulo
    uint16_t angle = Wire.read();        // Lee el primer byte (0x0E)
    angle = (angle << 8) | Wire.read();  // Lee el segundo byte (0x0F) y combina

    return angle & 0x0FFF; // Aplicar máscara para obtener solo los 12 bits del ángulo

}

// Transformar valor ADC a grados
double rawToDeg(uint16_t value){
  return (float)value*360/4095;
}

// === RUTINA PARA LEER ENCODER ===
float leerEncoder(uint8_t chan_encoder)
{
  uint8_t num_encoder = chan_encoder + 1; // Numero de encoder (1, 2 o 3)
  tcaselect(chan_encoder); // Selecciona el canal del multiplexor para el encoder 1
  uint16_t rawValue = readEncoder(); // Valor crudo 12 bits
  float angleDegrees = rawToDeg(rawValue);
  
  uint8_t reduction = reductionQ[num_encoder];

  float realAngle = angleDegrees / reduction; // Aplica la reducción correspondiente

  Serial.println("Encoder: "+String(num_encoder)+"-> "+String(realAngle)+"°");
  
  return realAngle; // Retorna el valor en grados
}

void encodersInfo()
{
  leerEncoder(0); // Encoder 1
  delay(50);
  leerEncoder(1); // Encoder 2
  delay(50);
  leerEncoder(2); // Encoder 3
  delay(50);
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