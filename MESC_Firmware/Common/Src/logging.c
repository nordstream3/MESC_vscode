#include "logging.h"
#include "motorinstance.h"
#include "usbd_cdc_if.h"
#include <stdio.h>



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
	sampled_vars.flags[sampled_vars.current_sample] = (_motor->hfi.inject_high_low_now == 1) | ((_motor->hfi.was_last_tracking == 1) << 1);
	sampled_vars.didq_d[sampled_vars.current_sample] = _motor->hfi.didq.d;
	sampled_vars.didq_q[sampled_vars.current_sample] = _motor->hfi.didq.q;
	sampled_vars.HFI_int_err[sampled_vars.current_sample] = _motor->hfi.HFI_int_err;

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
