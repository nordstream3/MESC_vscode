#ifndef LOGGING_H
#define LOGGING_H

#include <stdbool.h>
#include "hw_setup.h"

//#define LOGGING

#ifndef LOGLENGTH
#define LOGLENGTH 200
#define POST_ERROR_SAMPLES 		LOGLENGTH/4
#endif

#define LOG_INTERVAL 1
#define OSC_52_TAG_SIZE 7

// Forward declaration of MESC_motor_typedef
typedef struct MESC_motor_typedef MESC_motor_typedef;


//We want to log primarily Ia Ib Ic, Vd,Vq, phase angle, which gives us a complete picture of the machine state
//4 bytes per variable*6 variables*1000 = 24000bytes. Lowest spec target is F303CB with 48kB SRAM, so this is OK
typedef struct {
	float Vbus[LOGLENGTH];
	float Iu[LOGLENGTH];
	float Iv[LOGLENGTH];
	float Iw[LOGLENGTH];
	float Vd[LOGLENGTH];
	float Vq[LOGLENGTH];
	uint16_t angle[LOGLENGTH];

	// HFI logging added by JEO
	/*uint16_t flags[LOGLENGTH];
	float didq_d[LOGLENGTH];
	float didq_q[LOGLENGTH];
	float HFI_int_err[LOGLENGTH];
	uint32_t angle_flags[LOGLENGTH];*/

	uint32_t current_sample;

} sampled_vars_t;

extern volatile int print_samples_now, lognow;

extern sampled_vars_t sampled_vars;

void logVars(MESC_motor_typedef *_motor);
void printSamples(UART_HandleTypeDef *uart, DMA_HandleTypeDef *dma);

extern volatile uint32_t log_count;

uint32_t sample_init(MESC_motor_typedef *_motor);
void start_sample(uint32_t log_time);
void sample_n_bits_2_4_8();
void sample_n_bits();
char* getLogHeader();
uint8_t* getDataChunk();


#endif
