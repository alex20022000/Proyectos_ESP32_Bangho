#include <Arduino.h>
#include <Comunication.h>

void setup()
{
  initSerial();
}

void loop()
{
  readSerial();

  // Serial.println("haciendo otras cosas...");
}
