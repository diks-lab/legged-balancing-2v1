#pragma once
#include "Arduino.h"
using gpio_num_t = int;
inline int gpio_set_level(gpio_num_t pin, int value) { digitalWrite(pin, value); return 0; }
