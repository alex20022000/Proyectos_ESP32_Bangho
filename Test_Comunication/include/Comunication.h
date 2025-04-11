#pragma once
#include <Arduino.h>

// Buffer para almacenar la línea entrante
const byte MAX_LENGTH = 64;   // Longitud máxima del comando
char inputBuffer[MAX_LENGTH]; // Array para guardar caracteres
byte idx = 0;                 // Índice actual en el buffer
bool echoEnabled = true;      // Variable para habilitar/deshabilitar el eco

// Prototipos
void initSerial();                    // Inicializa la comunicación serie
void processCommand(const char *cmd); // Procesa el comando recibido
void readSerial();                    // Lee los datos del puerto serie

void initSerial()
{
    Serial.begin(115200); // Inicializa la comunicación serie a 9600 baudios
    while (!Serial)
    {
        ; // Espera a que el puerto serie esté listo (solo en placas con USB nativo)
    }
    Serial.println("Puerto serie inicializado correctamente.");
    Serial.println("Esperando comandos...");
}

void readSerial()
{
    // Comprueba si hay datos disponibles en el puerto serie
    while (Serial.available() > 0)
    {
        char c = Serial.read(); // Lee un carácter
        // Eco inmediato solo si está habilitado
        if (echoEnabled)
        {
            Serial.print(c);
        }
        // Si recibe el carácter de nueva línea, termina la cadena y la procesa
        if (c == '\n')
        {
            inputBuffer[idx] = '\0'; // Añade terminador de cadena
            processCommand(inputBuffer);
            idx = 0; // Reinicia el índice para la siguiente línea
        }
        else if (c != '\r')
        {
            // Sólo añade al buffer si no es retorno de carro y hay espacio
            if (idx < (MAX_LENGTH - 1))
            {
                inputBuffer[idx++] = c;
            }
            // Si el buffer se llena, lo terminas y procesas
            else
            {
                inputBuffer[idx] = '\0';
                processCommand(inputBuffer);
                idx = 0;
            }
        }
        // Si c == '\r', lo ignoramos
    }
}

void processCommand(const char *cmd)
{
    Serial.print("Comando recibido: ");
    Serial.println(cmd);

    // Ejemplo de parseo sencillo
    if (strcmp(cmd, "LED ON") == 0)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("LED encendido");
    }
    else if (strcmp(cmd, "LED OFF") == 0)
    {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("LED apagado");
    }
    else
    {
        Serial.println("Comando no reconocido");
    }

    // Comandos para controlar el eco
    if (strcmp(cmd, "ECHO ON") == 0)
    {
        echoEnabled = true;
        Serial.println("Echo habilitado.");
        return;
    }
    else if (strcmp(cmd, "ECHO OFF") == 0)
    {
        echoEnabled = false;
        Serial.println("Echo deshabilitado.");
        return;
    }
}