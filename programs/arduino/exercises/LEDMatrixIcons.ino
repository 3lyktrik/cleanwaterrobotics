// Εικονίδια για LED matrix
#include "Arduino_LED_Matrix.h"
ArduinoLEDMatrix matrix;
const uint32_t forward[] = {
  0x400e01f,
  0x3f80e00,
  0xe00e00e0
};
const uint32_t backward[] = {
  0xe00e00e,
  0xe03f81,
  0xf00e0040
};
const uint32_t stop[] = {
  0x2642f41f,
  0x830c3fc1,
  0xf8090090
};
const uint32_t happy[] = {
  0x19819,
  0x80000001,
  0x81f8000
};

void setup()
{
  matrix.begin();
}

void loop()
{
  matrix.loadFrame(happy);
  delay(2000);
  matrix.loadFrame(forward);
  delay(2000);
  matrix.loadFrame(stop);
  delay(2000);
  matrix.loadFrame(backward);
  delay(2000);
}
