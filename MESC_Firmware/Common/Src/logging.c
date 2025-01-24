#include "logging.h"
#include "motorinstance.h"
#include "usbd_cdc_if.h"
#include <stdio.h>

#define SMPL_CNT 4
float oldValues[SMPL_CNT];
float scale[SMPL_CNT];
float* var_ptr[SMPL_CNT];
const unsigned char header[] = "<header 5bits> Vbus f10,Iu f10,Iv f10,Iw f10"; //,Vd f10,Vq f10,didq_d f10,didq_q f10";

// Define the number of bits for encoding
#define BITS 5

#define BUFFER_SIZE_MAX_BYTES 90
 //((SMPL_CNT/2) * 44)
#define SAMPLINGS (BUFFER_SIZE_MAX_BYTES * 8) / (SMPL_CNT*BITS)
#define BUFFER_SIZE_BITS SAMPLINGS*SMPL_CNT*BITS

// Double buffering
uint8_t sample_buf1[BUFFER_SIZE_BITS/8];
uint8_t sample_buf2[BUFFER_SIZE_BITS/8];
uint8_t* buffer_active;
uint8_t* buffer_copy;
uint8_t sample_idx;
bool data_ready;

volatile uint32_t log_count = 0;

uint8_t sample_init(MESC_motor_typedef *_motor, uint32_t log_time) {
	
	var_ptr[0] = &_motor->Conv.Vbus; scale[0] = 10;
	var_ptr[1] = &_motor->Conv.Iu; scale[1] = 10;
	var_ptr[2] = &_motor->Conv.Iv; scale[2] = 10;
	var_ptr[3] = &_motor->Conv.Iw; scale[3] = 10;
	/*var_ptr[4] = &_motor->hfi.didq.d; scale[4] = 10;
	var_ptr[5] = &_motor->hfi.didq.q; scale[5] = 10;
	var_ptr[6] = &_motor->FOC.Vdq.d; scale[6] = 10;
	var_ptr[7] = &_motor->FOC.Vdq.q; scale[7] = 10;*/
	sample_idx = 0;
	data_ready = false;
	memset(oldValues, 0, SMPL_CNT*sizeof(float));
	memset(sample_buf1, 0, BUFFER_SIZE_BITS/8);
	memset(sample_buf2, 0, BUFFER_SIZE_BITS/8);
	buffer_active = sample_buf1;
	buffer_copy = sample_buf2;
	log_count = log_time;
	return BUFFER_SIZE_BITS/8;
}

/**
 * sample_4bit - Sample real-time float variables in 4 bit structure
 *
 * Called from interrupt "thread"
 */
void sample_4bit() {

	for (uint8_t i=0; i<SMPL_CNT; i++) {

		// Difference between current and old
		int32_t diff = (int32_t)(scale[i] * (*(var_ptr[i]) - oldValues[i]));

		uint8_t d;
		if (diff < 0) {
			if (diff < -8)
				diff = -8;
			d = ((uint8_t)(-diff) ^ 0x0F) + 1; // XOR only with lower 4 bits
		}
		else {
			if (diff > 7)
				diff = 7;
			d = (uint8_t)diff;
		}

		// Store in 8-bit
		if (i%2) {
			buffer_active[sample_idx + i/2] |= d << 4;
			sample_idx++;
		}
		else
			buffer_active[sample_idx + i/2] = d;

		// Update old values
		oldValues[i] += diff / scale[i];
	}

	if (sample_idx == BUFFER_SIZE_BITS/8) {
		// Swap buffers
		uint8_t* buffer_tmp = buffer_copy;
		buffer_copy = buffer_active;
		buffer_active = buffer_tmp;
		sample_idx = 0;
		data_ready = true;
	}
}

/**
 * sample_n_bits - Sample real-time float variables with variable bit encoding
 *
 * Called from interrupt
 */
#define MAX_DIFF ((1 << (BITS - 1)) - 1)
#define MIN_DIFF (-(1 << (BITS - 1)))
#define MASK ((1 << BITS) - 1)

void sample_n_bits() {
    static uint16_t bit_pos = 0; // Track bit position within the buffer

    for (uint8_t i = 0; i < SMPL_CNT; i++) {
        // Difference between current and old
        int32_t diff = (int32_t)(scale[i] * (*(var_ptr[i]) - oldValues[i]));

        // Clamp the difference within the range of -MAX_DIFF to MAX_DIFF
        if (diff < MIN_DIFF)
            diff = MIN_DIFF;
        else if (diff > MAX_DIFF)
            diff = MAX_DIFF;

        // Encode the difference into the desired number of bits
        uint16_t encoded;
        if (diff < 0) {
            encoded = ((uint16_t)(-diff) ^ MASK) + 1; // Two's complement for negative
        } else {
            encoded = (uint16_t)diff;
        }

        // Write the encoded value into the buffer
        uint16_t remaining_bits = BITS;
        while (remaining_bits > 0) {
            uint16_t byte_idx = bit_pos / 8;            // Current byte index
            uint8_t bit_offset = bit_pos % 8;          // Bit offset in the current byte
            uint8_t available_bits = 8 - bit_offset;   // Bits available in the current byte

            // Number of bits to write in the current byte
            uint8_t bits_to_write = (remaining_bits < available_bits) ? remaining_bits : available_bits;

			// Clear bits
			buffer_active[byte_idx] &= ~(((1 << bits_to_write) - 1) << bit_offset);

            // Write the bits to the current byte
            buffer_active[byte_idx] |= (encoded & ((1 << bits_to_write) - 1)) << bit_offset;

            // Shift the encoded value to remove the bits we've written
            encoded >>= bits_to_write;

            // Update positions
            bit_pos += bits_to_write;
            remaining_bits -= bits_to_write;
        }

        // Update old values
        oldValues[i] += diff / scale[i];
    }

    // Check if the buffer is full
    if (bit_pos >= BUFFER_SIZE_BITS) {
        // Swap buffers
        uint8_t* buffer_tmp = buffer_copy;
        buffer_copy = buffer_active;
        buffer_active = buffer_tmp;
        bit_pos = 0;
        data_ready = true;
    }
}

const unsigned char* getLogHeader() {
	return header;
}

bool getDataChunk(uint8_t** data) {
	if (!data_ready)
		return false;
	*data = buffer_copy;
	data_ready = false;
	return true;
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
	/*sampled_vars.flags[sampled_vars.current_sample] = (_motor->hfi.inject_high_low_now == 1) | ((_motor->hfi.was_last_tracking == 1) << 1);
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
