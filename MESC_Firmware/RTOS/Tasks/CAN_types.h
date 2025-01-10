/*
 * CAN_types.h
 *
 *  Created on: 14.04.2023
 *      Author: jensk
 */

#ifndef CAN_TYPES_H_
#define CAN_TYPES_H_



#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "queue.h"

typedef enum{
	NODE_TYPE_ESC,
	NODE_TYPE_DASH
}node_type;

#ifdef HAL_CAN_MODULE_ENABLED

typedef struct {
	uint16_t message_id;
	uint8_t sender;
	uint8_t receiver;
	uint8_t len;
	uint8_t buffer[8];
}TASK_CAN_packet;

typedef struct {
	TaskHandle_t rx_task_handle;
	TaskHandle_t tx_task_handle;
	CAN_HandleTypeDef * hw;
	uint16_t node_id;
	uint16_t remote_node_id;
	uint32_t stream_dropped;
	char short_name[9];
	QueueHandle_t rx_queue;
	uint32_t rx_dropped;
	QueueHandle_t tx_queue;
}TASK_CAN_handle;


typedef struct _CAN_NODES_{
	uint32_t id;
	char short_name[9];
	uint32_t last_seen;
	void * data;
	node_type type;
}TASK_CAN_node;

#endif
#endif /* CAN_TYPES_H_ */
