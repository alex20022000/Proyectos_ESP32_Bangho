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
#define ENDSTOP_SW1_PIN 36
#define ENDSTOP_SW2_PIN 34
#define ENDSTOP_SW3_PIN 35

// Variables globales para estados de finales de carrera
volatile bool end_sw1 = false;
volatile bool end_sw2 = false;
volatile bool end_sw3 = false;

// Variable que guarda el salto de angulo actual
float stepAngle = 1.8; // Valor por defecto (Full Step)

// Configuracion de interrupciones para los finales de carrera
// attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW1_PIN), int_sw1, CHANGE);
// attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW2_PIN), int_sw2, CHANGE);
// attachInterrupt(digitalPinToInterrupt(ENDSTOP_SW3_PIN), int_sw3, CHANGE);

// Prototipos
void home();
void moveMotor(int motor, int steps);
void moverPasos(int stepPin, int dirPin, int pasos);
void setMicrostepping(int mode);
void procesarComandoStep(String command);
void mostrarMenuMicrostepping();

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
  setMicrostepping(0); // Full Step (1/1)
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
    else if (command.startsWith("step"))
    {
      procesarComandoStep(command);
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
    delayMicroseconds(15000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(15000);
  }
  Serial.println("✅ Movimiento completado.");
}

// === FUNCION PARA CAMBIAR EL MODO DE MICROSTEPPING ===
void setMicrostepping(int mode)
{
  const bool modes[6][3] = {
      {LOW, LOW, LOW},   // Full Step (1/1)
      {HIGH, LOW, LOW},  // Half Step (1/2)
      {LOW, HIGH, LOW},  // 1/4 Step
      {HIGH, HIGH, LOW}, // 1/8 Step
      {LOW, LOW, HIGH},  // 1/16 Step
      {HIGH, LOW, HIGH}  // 1/32 Step
  };

  const float stepAngles[6] = {1.8, 0.9, 0.45, 0.225, 0.1125, 0.05625};

  if (mode < 0 || mode > 5)
  {
    Serial.println("❌ Modo inválido. Use un valor entre 0 y 5.");
    return;
  }

  // Configurar los pines M0, M1 y M2
  digitalWrite(M0_PIN, modes[mode][0]);
  digitalWrite(M1_PIN, modes[mode][1]);
  digitalWrite(M2_PIN, modes[mode][2]);

  // Actualizar el ángulo por paso
  stepAngle = stepAngles[mode];

  Serial.print("✅ Microstepping ajustado a: ");
  Serial.println(mode);
  Serial.print("📏 Paso angular actual: ");
  Serial.print(stepAngle);
  Serial.println("°");
}

// === FUNCION PARA PROCESAR EL COMANDO <STEP> ===
void procesarComandoStep(String command) {
  if (command == "step") {
      mostrarMenuMicrostepping();
  } else {
      int mode;
      if (sscanf(command.c_str(), "step %d", &mode) == 1) {
          setMicrostepping(mode);
      } else {
          Serial.println("❌ Comando inválido. Use: step <modo> (0 a 5)");
      }
  }
}

// === FUNCION QUE MUESTRA EL MENU DE MODIFICACION DE MICROSTEPPING ===
void mostrarMenuMicrostepping() {
  Serial.println("\n🔧 Seleccione la resolución deseada:");
  Serial.println("0. Full Step [1/1 ; 1.8°]");
  Serial.println("1. Half Step [1/2 ; 0.9°]");
  Serial.println("2. 1/4 Step  [1/4 ; 0.45°]");
  Serial.println("3. 1/8 Step  [1/8 ; 0.225°]");
  Serial.println("4. 1/16 Step [1/16 ; 0.1125°]");
  Serial.println("5. 1/32 Step [1/32 ; 0.05625°]");
  Serial.println("👉 Ingrese el número de la opción:");
}
