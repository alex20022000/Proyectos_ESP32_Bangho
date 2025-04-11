#include <Comunication.h>

// Buffer para almacenar la línea entrante
static const byte MAX_LENGTH = 64;
static char inputBuffer[MAX_LENGTH];
static byte idx = 0;

// Control de eco
static bool echoEnabled = true;

void initSerial() {
    Serial.begin(115200);
    while (!Serial) { }
    Serial.println("Puerto serie inicializado.");
    Serial.println("Esperando comandos...");
}

void readSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        // Manejo de Backspace (opcional)
//      if (c == '\b' || c == 127) {
//          if (idx > 0) {
//              idx--;
//              if (echoEnabled) Serial.print("\b \b");
//          }
//          continue;
//      }

        // Eco inmediato
        if (echoEnabled) {
            Serial.print(c);
        }

        // Fin de línea
        if (c == '\n') {
            inputBuffer[idx] = '\0';
            processCommand(inputBuffer);
            idx = 0;
        }
        else if (c != '\r') {
            if (idx < MAX_LENGTH - 1) {
                inputBuffer[idx++] = c;
            } else {
                // Buffer lleno sin '\n'
                inputBuffer[idx] = '\0';
                processCommand(inputBuffer);
                idx = 0;
            }
        }
    }
}

void processCommand(const char* cmd) {
    // Línea de control de eco primero
    if (strcmp(cmd, "ECHO ON") == 0) {
        echoEnabled = true;
        Serial.println("\nEcho habilitado.");
        return;
    }
    if (strcmp(cmd, "ECHO OFF") == 0) {
        echoEnabled = false;
        Serial.println("\nEcho deshabilitado.");
        return;
    }

    // Comando "home"
    if (strcmp(cmd, "home") == 0) {
        Serial.println("\nEjecutando home()");
        home();  // Asegúrate de tener `extern void home();` en algún header común
        return;
    }

    // Comando "move <motor> <pasos>"
    if (strncmp(cmd, "move ", 5) == 0) {
        int motor, pasos;
        if (sscanf(cmd, "move %d %d", &motor, &pasos) == 2) {
            Serial.printf("\nMove motor %d pasos %d\n", motor, pasos);
            moveMotor(motor, pasos);  // Igual, declara extern void moveMotor(...)
        } else {
            Serial.println("\n❌ Formato inválido. Use: move <motor> <pasos>");
        }
        return;
    }

    // Comando "step ..."
    if (strncmp(cmd, "step ", 5) == 0) {
        Serial.println("\nProcesando step...");
        // procesarComandoStep(cmd);
        return;
    }

    // Comando "ang ..."
    if (strncmp(cmd, "ang ", 4) == 0) {
        Serial.println("\nProcesando ang...");
        // procesarComandoAng(cmd);
        return;
    }

    // Comando "delta ..."
    if (strncmp(cmd, "delta ", 6) == 0) {
        Serial.println("\nProcesando delta...");
        // procesarComandoDelta(cmd);
        return;
    }

    // Comando "info"
    if (strcmp(cmd, "info") == 0) {
        Serial.println("\nMostrando info...");
        // mostrarInfo();
        return;
    }

    // Comando "switch"
    if (strcmp(cmd, "switch") == 0) {
        Serial.println("\nCapturando endstops...");
        // captureEndstopStates();
        return;
    }

    // Si no coincide ninguno
    Serial.println("\nComando no reconocido");
}
