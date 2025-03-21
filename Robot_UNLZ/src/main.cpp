#include <Arduino.h>

#define STEP_PIN_Q1 14
#define STEP_PIN_Q2 16
#define STEP_PIN_Q3 19
#define DIR_PIN_Q1 13
#define DIR_PIN_Q2 4
#define DIR_PIN_Q3 18
#define ENDSTOP_SW1_PIN 36
#define ENDSTOP_SW2_PIN 34
#define ENDSTOP_SW3_PIN 35

// Variables globales para estados de finales de carrera
volatile bool end_sw1 = false;
volatile bool end_sw2 = false;
volatile bool end_sw3 = false;

// Configuracion de interrupciones para los finales de carrera
// attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW1_PIN), int_sw1, CHANGE);
// attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW2_PIN), int_sw2, CHANGE);
// attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW3_PIN), int_sw3, CHANGE);

// Prototipos
void home();
void moveMotor(int motor, int steps);
void moverPasos(int stepPin, int dirPin, int pasos);

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
    else if (command == "ang")
    {
      // Mover a un ángulo
    }
    else
    {
      Serial.println("Command unrrecognized");
    }
  }
}

// Realizar el homming
void home()
{
  Serial.println("Checking endstops states...");

  // Config direcciones hacia finales de carrera
  digitalWrite(DIR_PIN_Q1, HIGH);
  digitalWrite(DIR_PIN_Q2, HIGH);
  digitalWrite(DIR_PIN_Q3, HIGH);

  // Verificar si algun final de carrera se encuentra pisado
  bool sw1 = digitalRead(ENDSTOP_SW1_PIN);
  bool sw2 = digitalRead(ENDSTOP_SW2_PIN);
  bool sw3 = digitalRead(ENDSTOP_SW3_PIN);

  if (sw1 || sw2 || sw3)
  {
    // Retroceso hasta que se liberen los finales de carrera
    if (sw1)
    {
      Serial.println("SW1 Pressed, going back Q1...");
      digitalWrite(DIR_PIN_Q1, LOW); // cambiar direccion para alejarse
      for (int i = 0; i < 11; i++)   // 10 pasos
      {
        digitalWrite(STEP_PIN_Q1, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN_Q1, LOW);
        delayMicroseconds(500);
      }
    }
    if (sw2)
    {
      Serial.println("SW2 Pressed, going back Q2...");
      digitalWrite(DIR_PIN_Q2, LOW); // cambiar direccion para alejarse
      for (int i = 0; i < 11; i++)   // 10 pasos
      {
        digitalWrite(STEP_PIN_Q2, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN_Q2, LOW);
        delayMicroseconds(500);
      }
    }
    if (sw3)
    {
      Serial.println("SW3 Pressed, going back Q3...");
      digitalWrite(DIR_PIN_Q3, LOW); // cambiar direccion para alejarse
      for (int i = 0; i < 11; i++)   // 10 pasos
      {
        digitalWrite(STEP_PIN_Q3, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN_Q3, LOW);
        delayMicroseconds(500);
      }
    }
  }
  Serial.println("Checking endstops OK...");
  // A partir de aca, iniciar la rutina de homing
}

// === FUNCION PARA MOVER UN MOTOR ===
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
  digitalWrite(dirPin, (steps > 0) ? HIGH : LOW);
  
  // Mover la cantidad de pasos solicitada
  moverPasos(stepPin, dirPin, abs(steps));
}

// === FUNCION PARA MOVER UN NUMERO DE PASOS ===
void moverPasos(int stepPin, int dirPin, int pasos)
{
  for (int i = 0; i < pasos; i++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  Serial.println("✅ Movimiento completado.");
}