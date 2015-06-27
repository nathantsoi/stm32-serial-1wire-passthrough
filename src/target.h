#pragma once

#define STM32F303xC
#define STM32F3DISCOVERY
#define USE_SERIAL_1WIRE
// How many escs does this board support?
#define ESC_COUNT 6
// STM32F3DISCOVERY TX - PD2 connects to UART RX
#define S1W_TX_GPIO         GPIOD
#define S1W_TX_PIN          GPIO_Pin_2
#define S1W_TX_PERIPH       RCC_AHBPeriph_GPIOD
// STM32F3DISCOVERY RX - PD0 connects to UART TX
#define S1W_RX_GPIO         GPIOD
#define S1W_RX_PIN          GPIO_Pin_0
#define S1W_RX_PERIPH       RCC_AHBPeriph_GPIOD
