/*
 **
 ******************************************************************************
 * @file           : task_overlay.c
 * @brief          : Overlay realtime data in terminal
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Jens Kerrinnes.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 ******************************************************************************
 *In addition to the usual 3 BSD clauses, it is explicitly noted that you
 *do NOT have the right to take sections of this code for other projects
 *without attribution and credit to the source. Specifically, if you copy into
 *copyleft licenced code without attribution and retention of the permissive BSD
 *3 clause licence, you grant a perpetual licence to do the same regarding turning sections of your code
 *permissive, and lose any rights to use of this code previously granted or assumed.
 *
 *This code is intended to remain permissively licensed wherever it goes,
 *maintaining the freedom to distribute compiled binaries WITHOUT a requirement to supply source.
 *
 *This is to ensure this code can at any point be used commercially, on products that may require
 *such restriction to meet regulatory requirements, or to avoid damage to hardware, or to ensure
 *warranties can reasonably be honoured.
 ******************************************************************************/


#include <stdlib.h>
#include <string.h>

#include "task_overlay.h"
#include "task_cli.h"
#include "task_can.h"
#include "logging.h"
#include "cmsis_os.h"
#include "motorinstance.h"
#include <math.h>
#include <stdio.h>



/* RTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


/* ------------------------------------------------------------------------ */
/*
 * Place user included headers, defines and task global data in the
 * below merge region section.
 */
/* `#START USER_INCLUDE SECTION` */
#include "TTerm/Core/include/TTerm.h"
#ifndef DASH
#include "motor_control.h"
#endif

/* `#END` */
/* ------------------------------------------------------------------------ */
/*
 * User defined task-local code that is used to process commands, control
 * operations, and/or generrally do stuff to make the taks do something
 * meaningful.
 */
/* `#START USER_TASK_LOCAL_CODE` */


#define OVERLAY_OUTPUT_NONE			0
#define OVERLAY_OUTPUT_VT100 		1
#define OVERLAY_OUTPUT_CSV 			2
#define OVERLAY_OUTPUT_JSON			3


void show_overlay(TERMINAL_HANDLE * handle){
#ifndef DASH
	MESC_motor_typedef * motor_curr = &mtr[0];

	TERM_sendVT100Code(handle, _VT100_CURSOR_SAVE_POSITION,0);
	TERM_sendVT100Code(handle, _VT100_CURSOR_DISABLE,0);

	uint8_t row_pos = 1;
	uint8_t col_pos = 90;
	TERM_Box(handle, row_pos, col_pos, row_pos + 6, col_pos + 31);
	TERM_setCursorPos(handle, row_pos + 1, col_pos + 1);
	ttprintf("Bus Voltage:       %10.1fV", (double)motor_curr->Conv.Vbus);

	TERM_setCursorPos(handle, row_pos + 2, col_pos + 1);
	ttprintf("Bus Current:      %10.2f A", (double)motor_curr->FOC.Ibus);

	TERM_setCursorPos(handle, row_pos + 3, col_pos + 1);
	ttprintf("Speed:         %10.2f ERPM", (double)(motor_curr->FOC.eHz*60.0f));

	TERM_setCursorPos(handle, row_pos + 4, col_pos + 1);
	ttprintf("Power:            %10.2f W", (double)motor_curr->FOC.currentPower.q);

	TERM_setCursorPos(handle, row_pos + 5, col_pos + 1);
	ttprintf("MESC status: ");

	switch(motor_curr->MotorState){
	case MOTOR_STATE_INITIALISING:
		ttprintf("     INITIALISING");
		break;
	case MOTOR_STATE_DETECTING:
		ttprintf("        DETECTING");
		break;
	case MOTOR_STATE_MEASURING:
		ttprintf("        MEASURING");
		break;
	case MOTOR_STATE_ALIGN:
		ttprintf("            ALIGN");
		break;
	case MOTOR_STATE_OPEN_LOOP_STARTUP:
		ttprintf("       OL STARTUP");
		break;
	case MOTOR_STATE_OPEN_LOOP_TRANSITION:
		ttprintf("    OL TRANSITION");
		break;
	case MOTOR_STATE_TRACKING:
		ttprintf("         TRACKING");
		break;
	case MOTOR_STATE_RUN:
		ttprintf("              RUN");
		break;
	case MOTOR_STATE_GET_KV:
		ttprintf("           GET KV");
		break;
	case MOTOR_STATE_TEST:
		ttprintf("             TEST");
		break;
	case MOTOR_STATE_ERROR:
		ttprintf("            ERROR");
		break;
	case MOTOR_STATE_RECOVERING:
		ttprintf("       RECOVERING");
		break;
	case MOTOR_STATE_IDLE:
		ttprintf("             IDLE");
		break;
	case MOTOR_STATE_SLAMBRAKE:
		ttprintf("        SLAMBRAKE");
		break;
	case MOTOR_STATE_RUN_BLDC:
		ttprintf("         RUN BLDC");
		break;
	}


	TERM_sendVT100Code(handle, _VT100_CURSOR_RESTORE_POSITION,0);
	TERM_sendVT100Code(handle, _VT100_CURSOR_ENABLE,0);
#endif

}

uint32_t format_json(TERMINAL_HANDLE * handle, TermVariableDescriptor * desc, char * buffer, int32_t len){
	uint32_t written=0;
	uint32_t bytes_written=0;
	if(desc->type == TERM_VARIABLE_CHAR || desc->type == TERM_VARIABLE_STRING){
		written = snprintf(buffer, len, "\"%s\":\"", desc->name);
	}else{
		written = snprintf(buffer, len, "\"%s\":", desc->name);
	}
	buffer += written;
	len -= written;
	bytes_written += written;
	written = TERM_var2str(handle, desc, buffer, len);
	buffer += written;
	len -= written;
	bytes_written += written;
	if(desc->type == TERM_VARIABLE_CHAR || desc->type == TERM_VARIABLE_STRING){
		written = snprintf(buffer, len, "\",");
	}else{
		written = snprintf(buffer, len, ",");
	}
	len -= written;
	bytes_written += written;
	//}

	return bytes_written;
}

void show_overlay_json(TERMINAL_HANDLE * handle){

	char buffer[512];
	char * ptr = buffer;
	uint32_t bytes_left = sizeof(buffer);
	int32_t written;

	bool found = false;

	*ptr = '{';
	ptr++;
	bytes_left--;

	uint32_t currPos = 0;
	TermVariableDescriptor * head = handle->varHandle->varListHead;
	TermVariableDescriptor * currVar = handle->varHandle->varListHead->nextVar;
    for(;currPos < head->nameLength; currPos++){

    	if(currVar->flags & FLAG_TELEMETRY_ON){
    		written = format_json(handle, currVar, ptr, bytes_left);
    		bytes_left -= written;
    		ptr += written;
    		found = true;

    	}
    	currVar = currVar->nextVar;
    }
    if(found) ptr--; //remove comma

    written = snprintf(ptr,bytes_left,"}\r\n");
    bytes_left -= written;

    ttprintf(NULL, buffer, sizeof(buffer)-bytes_left);

}



/* `#END` */
/* ------------------------------------------------------------------------ */
/*
 * This is the main procedure that comprises the task.  Place the code required
 * to preform the desired function within the merge regions of the task procedure
 * to add functionality to the task.
 */
void task_overlay_TaskProc(void *pvParameters) {
	/*
	 * Add and initialize local variables that are allocated on the Task stack
	 * the the section below.
	 */
	/* `#START TASK_VARIABLES` */

    TERMINAL_HANDLE * handle = pvParameters;

    port_str * port = handle->port;

	/* `#END` */

	/*
	 * Add the task initialzation code in the below merge region to be included
	 * in the task.
	 */
	/* `#START TASK_INIT_CODE` */

	/* `#END` */


    for (;;) {
		/* `#START TASK_LOOP_CODE` */
        xSemaphoreTake(port->term_block, portMAX_DELAY);

        switch(port->overlay_handle.output_type){
			case OVERLAY_OUTPUT_VT100:
				show_overlay(handle);
				break;
			case OVERLAY_OUTPUT_JSON:
				show_overlay_json(handle);
				break;
        }

        xSemaphoreGive(port->term_block);

        vTaskDelay(pdMS_TO_TICKS(port->overlay_handle.delay));

	}
}

/*****************************************************************************
* Helper function for spawning the overlay task
******************************************************************************/
void start_overlay_task(TERMINAL_HANDLE * handle){

	port_str * port = handle->port;

    if (port->overlay_handle.task_handle == NULL) {

        xTaskCreate(task_overlay_TaskProc, "Overlay", 512, handle, osPriorityNormal, &port->overlay_handle.task_handle);

    }
}


/*****************************************************************************
* Helper function for killing the overlay task
******************************************************************************/
void stop_overlay_task(TERMINAL_HANDLE * handle){
	port_str * port = handle->port;
    if (port->overlay_handle.task_handle != NULL) {
        vTaskDelete(port->overlay_handle.task_handle);
    	port->overlay_handle.task_handle = NULL;
    }
}


/*****************************************************************************
*
******************************************************************************/
uint8_t CMD_status(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    if(argCount==0 || strcmp(args[0], "-?") == 0){
        ttprintf("Usage: status [start|stop]\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }

    port_str * port = handle->port;

	if(strcmp(args[0], "start") == 0){
		port->overlay_handle.delay = 500;  //ms
		port->overlay_handle.output_type = OVERLAY_OUTPUT_VT100;
		start_overlay_task(handle);
        return TERM_CMD_EXIT_SUCCESS;
	}
	if(strcmp(args[0], "json") == 0){
		port->overlay_handle.delay = 100;  //ms
		port->overlay_handle.output_type = OVERLAY_OUTPUT_JSON;
		start_overlay_task(handle);
		return TERM_CMD_EXIT_SUCCESS;
	}
	if(strcmp(args[0], "stop") == 0){
		port->overlay_handle.output_type = OVERLAY_OUTPUT_NONE;
		stop_overlay_task(handle);
        return TERM_CMD_EXIT_SUCCESS;
	}
    return TERM_CMD_EXIT_SUCCESS;
}


void log_mod(TERMINAL_HANDLE * handle, char * name, bool delete){
	uint32_t currPos = 0;
	TermVariableDescriptor * head = handle->varHandle->varListHead;
	TermVariableDescriptor * currVar = handle->varHandle->varListHead->nextVar;

	for(;currPos < head->nameLength; currPos++){

		if(strcmp(name, currVar->name)==0){
			if(delete){
				ttprintf("Removed [%s] from log list\r\n", name);
				TERM_clearFlag(currVar, FLAG_TELEMETRY_ON);
			}else{
				ttprintf("Added [%s] to log list\r\n", name);
				TERM_setFlag(currVar, FLAG_TELEMETRY_ON);
			}
		}
		currVar = currVar->nextVar;
	}
}

void print_array(TERMINAL_HANDLE * handle, char * name, void * array, uint32_t count, uint32_t start_pos, uint32_t type_size ,TermVariableType type){

	char buffer[512];
	char * ptr = buffer;
	uint32_t bytes_left = sizeof(buffer);
	int32_t written;

	uint32_t u_var = 0;
	int32_t i_var = 0;


	uint32_t currPos = start_pos;

	written = snprintf(ptr, bytes_left, "\"%s\":[", name);
	ptr += written;
	bytes_left -= written;



	for(uint32_t i=0;i<count;i++){

		switch(type){
			case TERM_VARIABLE_FLOAT_ARRAY:
				written = snprintf(ptr, bytes_left, "%.2f,", (double)((float*)array)[currPos]);
				break;
			case TERM_VARIABLE_UINT_ARRAY:
				switch(type_size){
					case 1:
						u_var = ((uint8_t*)array)[currPos];
						break;
					case 2:
						u_var = ((uint16_t*)array)[currPos];
						break;
					case 4:
						u_var = ((uint32_t*)array)[currPos];
						break;
				}

				written = snprintf(ptr, bytes_left, "%lu,", u_var);
				break;
			case TERM_VARIABLE_INT_ARRAY:
					switch(type_size){
						case 1:
							i_var = ((uint8_t*)array)[currPos];
							break;
						case 2:
							i_var = ((uint16_t*)array)[currPos];
							break;
						case 4:
							i_var = ((uint32_t*)array)[currPos];
							break;
					}

					written = snprintf(ptr, bytes_left, "%li,", i_var);
				break;
			default:
				break;
		}

		currPos++;
		if(currPos == count){
			currPos=0;
		}

		ptr += written;
		bytes_left -= written;
		if(i == count-1){
			*(ptr-1) = ']';	//remove trailing comma
		}

		if(bytes_left < 16 || i == count-1){

			ttprintf(NULL, buffer, sizeof(buffer)-bytes_left);
			ptr = buffer;
			bytes_left = sizeof(buffer);
		}

	}

}

void print_index(TERMINAL_HANDLE * handle, char * name, uint32_t count, float increment){

	char buffer[512];
	char * ptr = buffer;
	uint32_t bytes_left = sizeof(buffer);
	int32_t written;

	float index=0.0f;

	written = snprintf(ptr, bytes_left, "\"%s\":[", name);
	ptr += written;
	bytes_left -= written;

	for(uint32_t i=0;i<count;i++){

		written = snprintf(ptr, bytes_left, "%f,", (double)index);
		index += increment;

		ptr += written;
		bytes_left -= written;
		if(i == count-1){
			*(ptr-1) = ']';	//remove trailing comma
		}

		if(bytes_left < 16 || i == count-1){

			ttprintf(NULL, buffer, sizeof(buffer)-bytes_left);
			ptr = buffer;
			bytes_left = sizeof(buffer);
		}

	}

}


/**
 * base64_encode - Base64 encode
 * @src: Data to be encoded
 * @len: Length of the data to be encoded
 * @out_len: Pointer to output length variable, or %NULL if not used
 * Returns: Allocated buffer of out_len bytes of encoded data,
 * or %NULL on failure
 *
 * Caller is responsible for freeing the returned buffer. Returned buffer is
 * nul terminated to make it easier to use as a C string. The nul terminator is
 * not included in out_len.
 */
static const unsigned char base64_table[65] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t base64_encode(void *dest, const void *src, size_t n)
{
	unsigned char *pos;
	const unsigned char *end, *in;

	end = src + n;
	in = src;
	pos = dest;
	while (end - in >= 3) {
		*pos++ = base64_table[in[0] >> 2];
		*pos++ = base64_table[((in[0] & 0x03) << 4) | (in[1] >> 4)];
		*pos++ = base64_table[((in[1] & 0x0f) << 2) | (in[2] >> 6)];
		*pos++ = base64_table[in[2] & 0x3f];
		in += 3;
	}

	if (end - in) {
		*pos++ = base64_table[in[0] >> 2];
		if (end - in == 1) {
			*pos++ = base64_table[(in[0] & 0x03) << 4];
			*pos++ = '=';
		} else {
			*pos++ = base64_table[((in[0] & 0x03) << 4) | (in[1] >> 4)];
			*pos++ = base64_table[(in[1] & 0x0f) << 2];
		}
		*pos++ = '=';
	}

	*pos = '\0';

	return pos - (unsigned char*)dest;
}

size_t base64_encode_asm(void *dest, const void *src, size_t n) {
    unsigned char *pos = (unsigned char *)dest;
    const unsigned char *in = (const unsigned char *)src;
    const unsigned char *end = in + n;

    while (end - in >= 3) {
        uint32_t chunk = (in[0] << 16) | (in[1] << 8) | in[2];

        __asm volatile (
            "lsl r3, %[chunk], #6     \n"  // r3 = chunk << 6 (prepare for masking)
            "lsr r3, r3, #6           \n"  // r3 = chunk >> 6
            "and r0, %[chunk], #0xFC0000  \n"  // Extract first 6 bits
            "lsr r0, r0, #18          \n"
            "ldrb r0, [%[table], r0]  \n"
            "strb r0, [%[out]], #1    \n"

            "and r1, r3, #0x3F00      \n"  // Extract second 6 bits
            "lsr r1, r1, #8           \n"
            "ldrb r1, [%[table], r1]  \n"
            "strb r1, [%[out]], #1    \n"

            "and r2, r3, #0xFC        \n"  // Extract third 6 bits
            "lsr r2, r2, #2           \n"
            "ldrb r2, [%[table], r2]  \n"
            "strb r2, [%[out]], #1    \n"

            "and r3, %[chunk], #0x3F  \n"  // Extract fourth 6 bits
            "ldrb r3, [%[table], r3]  \n"
            "strb r3, [%[out]], #1    \n"
            : [out] "+r" (pos)
            : [chunk] "r" (chunk), [table] "r" (base64_table)
            : "r0", "r1", "r2", "r3", "memory"
        );

        in += 3;
    }

    if (end - in) {
        uint8_t tail[3] = {0, 0, 0};
        tail[0] = in[0];
        if (end - in == 2) tail[1] = in[1];

        uint32_t chunk = (tail[0] << 16) | (tail[1] << 8);

        __asm volatile (
            "and r0, %[chunk], #0xFC0000  \n"
            "lsr r0, r0, #18          \n"
            "ldrb r0, [%[table], r0]  \n"
            "strb r0, [%[out]], #1    \n"

            "and r1, %[chunk], #0x3F00      \n"
            "lsr r1, r1, #8           \n"
            "ldrb r1, [%[table], r1]  \n"
            "strb r1, [%[out]], #1    \n"

            "cmp %[len], #1           \n"
            "beq encode_padding       \n"

            "and r2, %[chunk], #0xFC        \n"
            "lsr r2, r2, #2           \n"
            "ldrb r2, [%[table], r2]  \n"
            "strb r2, [%[out]], #1    \n"
            "b end_encoding           \n"

        "encode_padding:            \n"
            "mov r2, #61              \n"  // ASCII '='
            "strb r2, [%[out]], #1    \n"

        "end_encoding:              \n"
            "mov r3, #61              \n"  // ASCII '='
            "strb r3, [%[out]], #1    \n"
            : [out] "+r" (pos)
            : [chunk] "r" (chunk), [table] "r" (base64_table), [len] "r" (end - in)
            : "r0", "r1", "r2", "r3", "memory"
        );
    }

    *pos = '\0';
    return pos - (unsigned char*)dest;
}

void log_fastloop(TERMINAL_HANDLE * handle){
/*#ifndef DASH
	lognow = 1;
	vTaskDelay(100);

	print_samples_now = 0;
	lognow = 0;
	vTaskDelay(10);

	unsigned char src[256];
	unsigned char dest[256];

	//uint32_t bytes_left = sizeof(buffer);
	//int32_t written;

	int i = sampled_vars.current_sample;

	// 4 bytes: float (f)
	// 2 bytes: int16 (h)
	// 2 bytes: uint16 (H)
	// 4 bytes: int32 (i)
	// 4 bytes: uint32 (I)

	ttprintf("\033]52;c;");
	const unsigned char header[] = "<header> time f,Vbus f,Iu f,Iv f,Iw f,Vd f,Vq f,angle H,flags H,didq_d f,didq_q f,mag45 f,HFI_int_err f";
	base64_encode(dest, header, strlen((const char*)header));
	ttprintf((const char*)dest);
	ttprintf("\a");

	float time = 0.0f;
	size_t sz;
	for(uint32_t j=0;j<LOGLENGTH;j++) {

		unsigned char *ptr = src;
		ttprintf("\033]52;c;");

		sz = sizeof(time);
		ptr = memcpy(ptr, &time, sz);
		ptr += sz;
		time += mtr[0].FOC.pwm_period;

		sz = sizeof(sampled_vars.Vbus[i]);
		ptr = memcpy(ptr, &sampled_vars.Vbus[i], sz);
		ptr += sz;

		sz = sizeof(sampled_vars.Iu[i]);
		ptr = memcpy(ptr, &sampled_vars.Iu[i], sz);
		ptr += sz;

		sz = sizeof(sampled_vars.Iv[i]);
		ptr = memcpy(ptr, &sampled_vars.Iv[i], sz);
		ptr += sz;

		sz = sizeof(sampled_vars.Iw[i]);
		ptr = memcpy(ptr, &sampled_vars.Iw[i], sz);
		ptr += sz;

		sz = sizeof(sampled_vars.Vd[i]);
		ptr = memcpy(ptr, &sampled_vars.Vd[i], sz);
		ptr += sz;

		sz = sizeof(sampled_vars.Vq[i]);
		ptr = memcpy(ptr, &sampled_vars.Vq[i], sz);
		ptr += sz;

		sz = sizeof(sampled_vars.angle[i]);
		ptr = memcpy(ptr, &sampled_vars.angle[i], sz);
		ptr += sz;

		// HFI logging added by JEO
		sz = sizeof(sampled_vars.flags[i]);
		ptr = memcpy(ptr, &sampled_vars.flags[i], sz);
		ptr += sz;

		sz = sizeof(sampled_vars.didq_d[i]);
		ptr = memcpy(ptr, &sampled_vars.didq_d[i], sz);
		ptr += sz;

		sz = sizeof(sampled_vars.didq_q[i]);
		ptr = memcpy(ptr, &sampled_vars.didq_q[i], sz);
		ptr += sz;

		float mag45 = sqrtf(sampled_vars.didq_d[i] * sampled_vars.didq_d[i] + sampled_vars.didq_q[i] * sampled_vars.didq_q[i]);
		sz = sizeof(mag45);
		ptr = memcpy(ptr, &mag45, sz);
		ptr += sz;

		sz = sizeof(sampled_vars.HFI_int_err[i]);
		ptr = memcpy(ptr, &sampled_vars.HFI_int_err[i], sz);
		ptr += sz;

		base64_encode(dest, src, ptr-src);
		ttprintf((const char*)dest);
		ttprintf("\a");

		i++;
		if(i == LOGLENGTH){
			i=0;
		}
	}


	vTaskDelay(100);
	lognow = 1;
#endif*/
}


void log_TaskProc2(void *pvParameters) {
	/*
	 * Add and initialize local variables that are allocated on the Task stack
	 * the the section below.
	 */
	/* `#START TASK_VARIABLES` */

    TERMINAL_HANDLE * handle = pvParameters;

    port_str * port = handle->port;

	/* `#END` */

	/*
	 * Add the task initialzation code in the below merge region to be included
	 * in the task.
	 */
	/* `#START TASK_INIT_CODE` */
	// 10 seconds of logging
	uint32_t log_max = (uint32_t)(mtr[0].FOC.pwm_frequency * 10.0f);

	//unsigned char encoded[256];
	MESC_motor_typedef * motor_curr = &mtr[0];
	uint8_t buf_size = sample_init(motor_curr);

	// 4 bytes: float (f)
	// 2 bytes: int16 (h)
	// 2 bytes: uint16 (H)
	// 4 bytes: int32 (i)
	// 4 bytes: uint32 (I)

	ttprintf("\033]52;c;");
	char* header = getLogHeader();
	//base64_encode(encoded, header, strlen((const char*)header));
	ttprintf((const char*)header);
	ttprintf("\a");

	//#define OSC_52_TAG_SIZE 7
	//memcpy(encoded, "\033]52;c;", OSC_52_TAG_SIZE);
	start_sample(log_max);

    while (log_count) {
		/* `#START TASK_LOOP_CODE` */

		// Wait for data chunk ready
		uint8_t* data = getDataChunk();

		if (log_count) {
			//size_t b64_buf_size = base64_encode(&encoded[OSC_52_TAG_SIZE], data, buf_size);
			//encoded[OSC_52_TAG_SIZE + b64_buf_size] = '\a';
			//putbuffer_usb(encoded, OSC_52_TAG_SIZE+b64_buf_size+1, port);

			putbuffer_usb(data-OSC_52_TAG_SIZE, OSC_52_TAG_SIZE+buf_size+1, port);
		}
	}

	// Task decides to terminate itself
	vTaskDelete(NULL);
}


bool flag_stream_log = false;
void log_TaskProc(void *pvParameters) {
	/*
	 * Add and initialize local variables that are allocated on the Task stack
	 * the the section below.
	 */
	/* `#START TASK_VARIABLES` */

    TERMINAL_HANDLE * handle = pvParameters;

    port_str * port = handle->port;

	/* `#END` */

	/*
	 * Add the task initialzation code in the below merge region to be included
	 * in the task.
	 */
	/* `#START TASK_INIT_CODE` */
	unsigned char src[256];
	unsigned char dest[256];

	// 4 bytes: float (f)
	// 2 bytes: int16 (h)
	// 2 bytes: uint16 (H)
	// 4 bytes: int32 (i)
	// 4 bytes: uint32 (I)

	// 10 seconds of logging
	uint32_t log_max = (uint32_t)(mtr[0].FOC.pwm_frequency * 10.0f);

	ttprintf("\033]52;c;");
	const unsigned char header[] = "<header> Vbus f,Iu f,Iv f,Iw f,Vd f,Vq f,angle H,flags H,didq_d f,didq_q f,mag45 f,HFI_int_err f";
	base64_encode(dest, header, strlen((const char*)header));
	ttprintf((const char*)dest);
	ttprintf("\a");

	// This only works when LOGLENGTH = 256 (size_of(uint8_t))
	const uint8_t log_len = 0xFF/2;
	uint8_t i_now = (uint8_t)sampled_vars.current_sample;
	uint8_t i_0 = i_now - log_len;
	uint8_t dist;
	/* `#END` */


    for (uint32_t j=0; j<log_max;) {
		/* `#START TASK_LOOP_CODE` */

		if (flag_stream_log == false)
			break;

		while (true) {
			i_now = (uint8_t)sampled_vars.current_sample;
			dist = i_now - i_0;
			if (dist >= log_len)
				break;
	        vTaskDelay(0);
		}

        xSemaphoreTake(port->term_block, portMAX_DELAY);

		size_t sz;
		for(uint8_t i=i_0; i!=i_now; i++) {

			unsigned char *ptr = src;
			ttprintf("\033]52;c;");

			sz = sizeof(sampled_vars.Vbus[i]);
			ptr = memcpy(ptr, &sampled_vars.Vbus[i], sz);
			ptr += sz;

			sz = sizeof(sampled_vars.Iu[i]);
			ptr = memcpy(ptr, &sampled_vars.Iu[i], sz);
			ptr += sz;

			sz = sizeof(sampled_vars.Iv[i]);
			ptr = memcpy(ptr, &sampled_vars.Iv[i], sz);
			ptr += sz;

			sz = sizeof(sampled_vars.Iw[i]);
			ptr = memcpy(ptr, &sampled_vars.Iw[i], sz);
			ptr += sz;

			sz = sizeof(sampled_vars.Vd[i]);
			ptr = memcpy(ptr, &sampled_vars.Vd[i], sz);
			ptr += sz;

			sz = sizeof(sampled_vars.Vq[i]);
			ptr = memcpy(ptr, &sampled_vars.Vq[i], sz);
			ptr += sz;

			sz = sizeof(sampled_vars.angle[i]);
			ptr = memcpy(ptr, &sampled_vars.angle[i], sz);
			ptr += sz;

			// HFI logging added by JEO
			/*sz = sizeof(sampled_vars.flags[i]);
			ptr = memcpy(ptr, &sampled_vars.flags[i], sz);
			ptr += sz;

			sz = sizeof(sampled_vars.didq_d[i]);
			ptr = memcpy(ptr, &sampled_vars.didq_d[i], sz);
			ptr += sz;

			sz = sizeof(sampled_vars.didq_q[i]);
			ptr = memcpy(ptr, &sampled_vars.didq_q[i], sz);
			ptr += sz;

			float mag45 = sqrtf(sampled_vars.didq_d[i] * sampled_vars.didq_d[i] + sampled_vars.didq_q[i] * sampled_vars.didq_q[i]);
			sz = sizeof(mag45);
			ptr = memcpy(ptr, &mag45, sz);
			ptr += sz;

			sz = sizeof(sampled_vars.HFI_int_err[i]);
			ptr = memcpy(ptr, &sampled_vars.HFI_int_err[i], sz);
			ptr += sz;*/

			base64_encode(dest, src, ptr-src);
			ttprintf((const char*)dest);
			ttprintf("\a");
		}

        xSemaphoreGive(port->term_block);

		j += dist;
		i_0 = i_now;
	}

	flag_stream_log = false;
}


/*****************************************************************************
*
******************************************************************************/
TaskHandle_t log_task_xHandle;
uint8_t CMD_log(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

    bool flag_list = false;
    bool flag_reset = false;
    bool flag_fastloop = false;
	
    port_str * port = handle->port;

	for(int i=0;i<argCount;i++){
		if(strcmp(args[i], "-l")==0){
			flag_list = true;
		}
		if(strcmp(args[i], "-r")==0){
			flag_reset = true;
		}
		if(strcmp(args[i], "-a")==0){
			if(i+1 < argCount){
				log_mod(handle, args[i+1], false);
			}
		}
		if(strcmp(args[i], "-d")==0){
			if(i+1 < argCount){
				log_mod(handle, args[i+1], true);
			}
		}
		if(strcmp(args[i], "-s")==0){
			if(i+1 < argCount){
				port->overlay_handle.delay = strtoul(args[i+1], NULL, 10);
			}
		}
		if(strcmp(args[i], "-?")==0){
			ttprintf("Usage: log [flags]\r\n");
			ttprintf("\t -a\t Add var to log\r\n");
			ttprintf("\t -d\t Delete var from log\r\n");
			ttprintf("\t -r\t Delete all vars from log\r\n");
			ttprintf("\t -l\t List vars\r\n");
			ttprintf("\t -s\t Log speed [ms]\r\n");
			ttprintf("\t -fl\t Print log\r\n");
			ttprintf("\t -st\t Continuous stream log\r\n");
			return TERM_CMD_EXIT_SUCCESS;
		}
		if(strcmp(args[i], "-fl")==0){
			flag_fastloop = true;
		}
		if(strcmp(args[i], "-st")==0){
			flag_stream_log = !flag_stream_log;
		}
	}

	if (flag_list){
		uint32_t currPos = 0;
		uint32_t count=0;
		TermVariableDescriptor * head = handle->varHandle->varListHead;
		TermVariableDescriptor * currVar = handle->varHandle->varListHead->nextVar;


		ttprintf("Datasources selected for log output:\r\n");

	    for(;currPos < head->nameLength; currPos++){

	    	if(currVar->flags & FLAG_TELEMETRY_ON){
	    		ttprintf("\t%u: %s\r\n", count, currVar->name);
	    		count++;
	    	}
	    	currVar = currVar->nextVar;
	    }
	    ttprintf("EOL\r\n");
	}

	if (flag_reset){
		uint32_t currPos = 0;
		TermVariableDescriptor * head = handle->varHandle->varListHead;
		TermVariableDescriptor * currVar = handle->varHandle->varListHead->nextVar;

		for(;currPos < head->nameLength; currPos++){

			if(currVar->flags & FLAG_TELEMETRY_ON){
				TERM_clearFlag(currVar, FLAG_TELEMETRY_ON);
			}
			currVar = currVar->nextVar;
		}
		ttprintf("Log list reset...\r\n");
	}

	if (flag_fastloop){
	#ifdef LOGGING
		log_fastloop(handle);
	#else
		ttprintf("Fastloop logging not enabled in firmware\r\n");
	#endif
	}

	if (flag_stream_log){
	#ifdef STREAM_LOGGING

		TaskHandle_t log_task_xHandle = NULL;
		BaseType_t res = xTaskCreate( log_TaskProc2, "log_task", 1024, handle, osPriorityNormal+2, &log_task_xHandle );
		configASSERT( log_task_xHandle );
		return res == pdPASS? TERM_CMD_EXIT_SUCCESS : TERM_CMD_EXIT_ERROR;
		//return CMD_start_log_task(handle);
	#else
		ttprintf("Stream logging not enabled in firmware\r\n");
	#endif
	}

    return TERM_CMD_EXIT_SUCCESS;
}
