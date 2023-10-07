#include <LibRobus.h>

float minmax(float minValue, float value, float maxValue) {
  return max(minValue, min(value, maxValue));
}