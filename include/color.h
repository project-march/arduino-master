#ifndef COLOR_H
#define COLOR_H
#include <Arduino.h>

struct Color
{
  Color(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue)
  {
  }

  const uint8_t r;
  const uint8_t g;
  const uint8_t b;
};

#endif  // COLOR_H