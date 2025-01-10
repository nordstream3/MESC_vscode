#ifndef INC_MESC_INTERFACE_H_
#define INC_MESC_INTERFACE_H_

#include "Tasks/task_cli.h"
#include "Tasks/task_can.h"

#define CAN_NAME "ESC_MP2"

void Interface_init(TERMINAL_HANDLE * handle);

uint8_t CMD_measure(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);

#ifdef HAL_CAN_MODULE_ENABLED
void TASK_CAN_telemetry_fast(TASK_CAN_handle * handle);
void TASK_CAN_telemetry_slow(TASK_CAN_handle * handle);
#endif

#endif
