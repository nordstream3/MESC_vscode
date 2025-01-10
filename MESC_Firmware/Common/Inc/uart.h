#ifndef MESC_UART_H
#define MESC_UART_H

#include "stm32fxxx_hal.h"

#define MESC_UART 0
#define MESC_USB  1

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void USB_CDC_Callback(uint8_t *buffer, uint32_t len);

void uart_init( void );

#endif
