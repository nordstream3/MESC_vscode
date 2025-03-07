#include "logging.h"
#include "motorinstance.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include "stm32f4xx.h"  // STM32F4 header for CMSIS

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

enum TypeEnum
{
    LOG_FLOAT,
	LOG_INT32,
	LOG_INT16,
    LOG_UINT16,
	LOG_UINT32,
	LOG_BOOL,
};

#define HEADER "<header bits=%d size=%d> injSign f1,wasLaTr f1,didq_q f10,FOCAngle f1"
#define SMPL_CNT 4
static const uint16_t scale[SMPL_CNT] = {1,1,10,1};
static const uint8_t logType[SMPL_CNT] = {LOG_INT16, LOG_INT32, LOG_FLOAT, LOG_UINT16};

// Define the number of bits for encoding
#define BITS 2

void* var_ptr[SMPL_CNT];
int32_t oldValues[SMPL_CNT];

#define BUFFER_SIZE_MAX_BYTES 90
#define SAMPLINGS (BUFFER_SIZE_MAX_BYTES * 8) / (SMPL_CNT*BITS)
#define BUFFER_SIZE_BITS SAMPLINGS*SMPL_CNT*BITS

char header[100];

#define OSC_52_TAG "\033]52;c;"

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// Double buffering
uint8_t sample_buf1[BUFFER_SIZE_BITS/8 + OSC_52_TAG_SIZE + 1];
uint8_t sample_buf2[BUFFER_SIZE_BITS/8 + OSC_52_TAG_SIZE + 1];
uint8_t* buffer_active;
uint8_t* buffer_copy;

volatile uint32_t log_count = 0;
static TaskHandle_t xTaskToNotify = NULL;

uint32_t sample_init(MESC_motor_typedef *_motor) { //, uint32_t log_time) {
	
    /* Store the handle of the calling task. */
    xTaskToNotify = xTaskGetCurrentTaskHandle();

	snprintf(header, sizeof(header), HEADER, BITS, BUFFER_SIZE_BITS/8);

	var_ptr[0] = &_motor->hfi.injection_sign;
	var_ptr[1] = &_motor->hfi.was_last_tracking;
	var_ptr[2] = &_motor->hfi.didq.q;
	var_ptr[3] = &_motor->FOC.FOCAngle;

    uint8_t bytes_used = SMPL_CNT*sizeof(int32_t);

    memset(oldValues, 0, bytes_used);
	oldValues[3] = _motor->FOC.FOCAngle;
	//memset(sample_buf1, 0, BUFFER_SIZE_BITS/8);
	//memset(sample_buf2, 0, BUFFER_SIZE_BITS/8);

	memcpy(sample_buf1, OSC_52_TAG, OSC_52_TAG_SIZE);
	memcpy(sample_buf2, OSC_52_TAG, OSC_52_TAG_SIZE);
    sample_buf1[BUFFER_SIZE_BITS/8 + OSC_52_TAG_SIZE] = '\a';
    sample_buf2[BUFFER_SIZE_BITS/8 + OSC_52_TAG_SIZE] = '\a';

    buffer_active = sample_buf1 + OSC_52_TAG_SIZE;
	buffer_copy = sample_buf2 + OSC_52_TAG_SIZE;
	//log_count = log_time;
	return BUFFER_SIZE_BITS/8;
}

void start_sample(uint32_t log_time) {
	log_count = log_time;
}

/**
 * sample_n_bits - Sample real-time variables with variable bit encoding
 *
 * Called from interrupt
 */
#define MAX_DIFF ((1 << (BITS - 1)) - 1)
#define MIN_DIFF (-(1 << (BITS - 1)))
#define MASK ((1 << BITS) - 1)

void sample_n_bits_2_4_8() {
    static uint16_t bit_pos = 0; // Track bit position within the buffer

    for (uint8_t i = 0; i < SMPL_CNT; i++) {

        // Calculate value change since last iteration
        int32_t new_val;
        void* ptr = var_ptr[i];
        switch(logType[i]) {
            case LOG_INT32:  new_val = (scale[i] * *(int32_t*)ptr); break;
            case LOG_INT16:  new_val = (scale[i] * *(int16_t*)ptr); break;
            case LOG_UINT16: new_val = (scale[i] * *(uint16_t*)ptr); break;
            case LOG_UINT32: new_val = (scale[i] * *(uint32_t*)ptr); break;
            case LOG_BOOL:   new_val = (scale[i] * *(bool*)ptr); break;
            default:         new_val = (scale[i] * *(float*)ptr);
        }

        // Clamp the difference
        int32_t diff;
        __asm volatile (
            "SUB %0, %2, %1               \n"
			"SSAT %0, #" STR(BITS) ", %0  \n"
            : "=r" (diff)
            : "r" (oldValues[i]), "r" (new_val)
            :
        );
        
        // Bit-packing
		uint16_t byte_idx = bit_pos >> 3;  // bit_pos / 8
		uint8_t bit_offset = bit_pos & 7;  // bit_pos % 8

		// Clear bits, because this is not done in double-buffer swap logic
		// ... maybe it should be, if it is faster?
		if (bit_offset==0)
			buffer_active[byte_idx] = 0;
		//buffer_active[byte_idx] &= ~(((1 << bits_to_write) - 1) << bit_offset);

		// Write the bits to the current byte
		buffer_active[byte_idx] |= (diff & ((1 << BITS) - 1)) << bit_offset;

		// Update positions
		bit_pos += BITS;

        // Update old values
        oldValues[i] += diff;
    }

    // Double-Buffer swap logic
    if (bit_pos >= BUFFER_SIZE_BITS) {
        uint8_t* tmp = buffer_copy;
        buffer_copy = buffer_active;
        buffer_active = tmp;
        bit_pos = 0;

        /* Notify the task for data transmission*/
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );          

        /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch  
        should be performed to ensure the interrupt returns directly to the highest  
        priority task. The macro used for this purpose is dependent on the port in  
        use and may be called portEND_SWITCHING_ISR(). */  
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );  
    }
}

void sample_n_bits() {
    static uint16_t bit_pos = 0; // Track bit position within the buffer

    for (uint8_t i = 0; i < SMPL_CNT; i++) {

        // Calculate value change since last iteration
        int32_t new_val;
        void* ptr = var_ptr[i];
        switch(logType[i]) {
            case LOG_INT32:  new_val = (scale[i] * *(int32_t*)ptr); break;
            case LOG_INT16:  new_val = (scale[i] * *(int16_t*)ptr); break;
            case LOG_UINT16: new_val = (scale[i] * *(uint16_t*)ptr); break;
            case LOG_UINT32: new_val = (scale[i] * *(uint32_t*)ptr); break;
            case LOG_BOOL:   new_val = (scale[i] * *(bool*)ptr); break;
            default:         new_val = (scale[i] * *(float*)ptr);
        }

        // Clamp the difference
        int32_t diff;
        __asm volatile (
            "SUB %0, %2, %1               \n"
			"SSAT %0, #" STR(BITS) ", %0  \n"
            : "=r" (diff)
            : "r" (oldValues[i]), "r" (new_val)
            :
        );
        
        // Bit-packing
        int32_t encoded = diff;
        uint16_t remaining_bits = BITS;
        while (remaining_bits) {
            uint16_t byte_idx = bit_pos >> 3;  // bit_pos / 8
            uint8_t bit_offset = bit_pos & 7;  // bit_pos % 8

            // Number of bits to write in the current byte
            uint8_t bits_to_write = 8 - bit_offset;
            if (remaining_bits < bits_to_write)
                bits_to_write = remaining_bits;

            // Clear bits, because this is not done in double-buffer swap logic
            // ... maybe it should be, if it is faster?
            if (bit_offset==0)
                buffer_active[byte_idx] = 0;
            //buffer_active[byte_idx] &= ~(((1 << bits_to_write) - 1) << bit_offset);

            // Write the bits to the current byte
            buffer_active[byte_idx] |= (encoded & ((1 << bits_to_write) - 1)) << bit_offset;

            // Shift the encoded value to remove the bits we've written
            encoded >>= bits_to_write;

            // Update positions
            bit_pos += bits_to_write;
            remaining_bits -= bits_to_write;
        }

        // Update old values
        oldValues[i] += diff;
    }

    // Double-Buffer swap logic
    if (bit_pos >= BUFFER_SIZE_BITS) {
        uint8_t* tmp = buffer_copy;
        buffer_copy = buffer_active;
        buffer_active = tmp;
        bit_pos = 0;

        /* Notify the task for data transmission*/
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );          

        /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch  
        should be performed to ensure the interrupt returns directly to the highest  
        priority task. The macro used for this purpose is dependent on the port in  
        use and may be called portEND_SWITCHING_ISR(). */  
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );  
    }
}

char* getLogHeader() {
	return header;
}

uint8_t* getDataChunk() {
    uint32_t ulNotificationValue = 0;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 1000 );  

    while (!ulNotificationValue) {
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    }
    return buffer_copy;
}


volatile int print_samples_now, lognow;

sampled_vars_t sampled_vars;

void logVars(MESC_motor_typedef *_motor)
{
	sampled_vars.Vbus[sampled_vars.current_sample] = _motor->Conv.Vbus;
	sampled_vars.Iu[sampled_vars.current_sample] = _motor->Conv.Iu;
	sampled_vars.Iv[sampled_vars.current_sample] = _motor->Conv.Iv;
	sampled_vars.Iw[sampled_vars.current_sample] = _motor->Conv.Iw;
	sampled_vars.Vd[sampled_vars.current_sample] = _motor->FOC.Vdq.d;
	sampled_vars.Vq[sampled_vars.current_sample] = _motor->FOC.Vdq.q;
	sampled_vars.angle[sampled_vars.current_sample] = _motor->FOC.FOCAngle;

	// HFI logging added by JEO
	/*sampled_vars.flags[sampled_vars.current_sample] = (_motor->hfi.injection_sign > 0) | ((_motor->hfi.was_last_tracking == 1) << 1);
	sampled_vars.didq_d[sampled_vars.current_sample] = _motor->hfi.didq.d;
	sampled_vars.didq_q[sampled_vars.current_sample] = _motor->hfi.didq.q;
	sampled_vars.HFI_int_err[sampled_vars.current_sample] = _motor->hfi.HFI_int_err;*/

	sampled_vars.current_sample++;
	if (sampled_vars.current_sample >= LOGLENGTH)
	{
		sampled_vars.current_sample = 0;
	}
}

int samples_sent;
uint32_t start_ticks;
extern DMA_HandleTypeDef hdma_usart3_tx;
void printSamples(UART_HandleTypeDef *uart, DMA_HandleTypeDef *dma)
{
#ifdef LOGGING
	uint8_t send_buffer[100];
	uint16_t length;
	if (print_samples_now)
	{
		print_samples_now = 0;
		lognow = 0;
		int current_sample_pos = sampled_vars.current_sample;
		start_ticks = HAL_GetTick();

		samples_sent = 0;

		while (samples_sent < LOGLENGTH)
		{
			HAL_Delay(1); // Wait 2ms, would be nice if we could poll for the CDC being free...
			samples_sent++;
			current_sample_pos++;

			if (current_sample_pos >= LOGLENGTH)
			{
				current_sample_pos = 0; // Wrap
			}

			length = snprintf((char *)send_buffer, sizeof(send_buffer), "%.2f%.2f,%.2f,%.2f,%.2f,%.2f,%d;\r\n",
							  (double)sampled_vars.Vbus[current_sample_pos],
							  (double)sampled_vars.Iu[current_sample_pos],
							  (double)sampled_vars.Iv[current_sample_pos],
							  (double)sampled_vars.Iw[current_sample_pos],
							  (double)sampled_vars.Vd[current_sample_pos],
							  (double)sampled_vars.Vq[current_sample_pos],
							  sampled_vars.angle[current_sample_pos]);
			if ((HAL_GetTick() - start_ticks) > 10000)
			{
				break;
			}
#ifdef MESC_UART_USB
			CDC_Transmit_FS(send_buffer, length);
#else

			HAL_UART_Transmit_DMA(uart, send_buffer, length);
			while (hdma_usart3_tx.State != HAL_DMA_STATE_READY)
			{ // Pause here
				//	while(hdma_usart3_tx.Lock != HAL_UNLOCKED){//Pause here
				__NOP();
			}
#endif
		}
		lognow = 1;
	}
#endif

}
