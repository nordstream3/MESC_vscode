#ifndef SIMPLE_COMS_H
#define SIMPLE_COMS_H

typedef struct {
	UART_HandleTypeDef *UART_handle;
	uint16_t some_thing;
	int16_t other_thing;
	char data[100];
	uint32_t len;
	uint32_t time;
	uint32_t period;
} COMS_data_t;


void SimpleComsInit(UART_HandleTypeDef *uart, COMS_data_t *coms_instance);

void SimpleComsProcess(COMS_data_t *coms_instance);

#endif
