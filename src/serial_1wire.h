#pragma once

#ifdef USE_SERIAL_1WIRE

typedef struct {
  uint32_t periph;
  GPIO_TypeDef* gpio;
  uint16_t pin;
} escHardware_t;

void usb1WirePassthrough(int8_t escIndex);
#endif
