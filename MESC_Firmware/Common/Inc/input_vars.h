#ifndef INPUT_VARS_H
#define INPUT_VARS_H

#include "foc.h"

typedef struct {

	///////////////////RCPWM//////////////////////
	uint32_t IC_duration; 	//Retrieve this from timer input capture CC1
	uint32_t IC_pulse; 		//Retrieve this from timer input capture CC2
	uint32_t pulse_recieved;

	uint32_t IC_duration_MAX;
	uint32_t IC_duration_MIN;
	uint32_t IC_pulse_MAX;
	uint32_t IC_pulse_MIN;
	uint32_t IC_pulse_MID;
	uint32_t IC_pulse_DEADZONE; //single sided; no response before MID +- this
	float RCPWM_gain[2][2];

	 uint32_t fCC1;
	 uint32_t fUPD;

/////////////////ADC///////////////
	uint32_t adc1_MIN; //Value below which response is zero
	uint32_t adc1_MAX; //Max throttle calculated at this point
	uint32_t adc1_OOR;
	float adc1_gain[2];

	uint32_t adc2_MIN;
	uint32_t adc2_MAX;
	uint32_t adc2_OOR;

	float adc2_gain[2];

	float ADC1_polarity;
	float ADC2_polarity;

	float UART_req;
	float UART_dreq;
	float RCPWM_req;
	float ADC1_req;
	float ADC2_req;
	float ADC12_diff_req;


	uint8_t remote_ADC_can_id;
	float remote_ADC1_req;
	float remote_ADC2_req;
	int32_t remote_ADC_timeout;


	uint16_t nKillswitch;
	uint16_t invert_killswitch;

	uint32_t input_options; //	0b...tuvwxyz where
							//	t is differential ADC,
							//	u is ADC1 remote,
							//	v is ADC2 remote
							//	w is UART,
							//	x is RCPWM,
							//	y is ADC1
							//	z is ADC2

	MESCiq_s max_request_Idq;
	MESCiq_s min_request_Idq;
} input_vars_t;

extern input_vars_t input_vars;

void InputInit();

void collectInputs(MESC_motor_typedef *_motor);

#endif
