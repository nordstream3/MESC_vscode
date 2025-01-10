#include "interrupt_adc.h"
#include "motorinstance.h"
#include "hw_setup.h"
#include "error.h"
#include "input_vars.h"
#include "conversions.h"

/*#include "hw_setup.h"
#include "motor_control.h"
#include "sin_lut.h"
#include "motor.h"
#include "temperature.h"
#include "error.h"
#include "position.h"
#include "motorinstance.h"
#include "observers.h"

#ifdef MESC_UART_USB
#include "usbd_cdc_if.h"
#endif


#include <math.h>
#include <stdlib.h>*/



// This should be the function needed to be added into the PWM interrupt
// for MESC to run. Ensure that it is followed by the clear timer update
// interrupt
void MESC_PWM_IRQ_handler(MESC_motor_typedef *_motor)
{
#ifdef FASTLED
	FASTLED->BSRR = FASTLEDIO;
#endif
	uint32_t cycles = CPU_CYCLES;
	if (_motor->mtimer->Instance->CR1 & 0x16)
	{ // Polling the DIR (direction) bit on the motor counter DIR = 1 = downcounting
		writePWM(_motor);
	}
	if (!(_motor->mtimer->Instance->CR1 & 0x16))
	{ // Polling the DIR (direction) bit on the motor counter DIR = 0 = upcounting
		RunHFI(_motor);
		writePWM(_motor);
	}
	_motor->FOC.cycles_pwmloop = CPU_CYCLES - cycles;

#ifdef FASTLED
	FASTLED->BSRR = FASTLEDIO << 16U;
#endif
}

void MESC_ADC_IRQ_handler(MESC_motor_typedef *_motor)
{
	fastLoop(_motor);
}

float maxIgamma;
uint16_t phasebalance;
void ADCConversion(MESC_motor_typedef *_motor)
{
	_motor->FOC.Idq_smoothed.d = (_motor->FOC.Idq_smoothed.d * 99.0f + _motor->FOC.Idq.d) * 0.01f;
	_motor->FOC.Idq_smoothed.q = (_motor->FOC.Idq_smoothed.q * 99.0f + _motor->FOC.Idq.q) * 0.01f;

	getRawADC(_motor);

	// Here we take the raw ADC values, offset, cast to (float) and use the
	// hardware gain values to create volt and amp variables
	// Convert the currents to real amps in SI units
	_motor->Conv.Iu =
		((float)_motor->Raw.Iu - _motor->offset.Iu) * g_hw_setup.Igain;
	_motor->Conv.Iv =
		((float)_motor->Raw.Iv - _motor->offset.Iv) * g_hw_setup.Igain;
	_motor->Conv.Iw =
		((float)_motor->Raw.Iw - _motor->offset.Iw) * g_hw_setup.Igain;
	_motor->Conv.Vbus =
		(float)_motor->Raw.Vbus * g_hw_setup.VBGain; // Vbus

	// Check for over limit conditions. We want this after the conversion so that the
	// correct overcurrent values are logged
	// VICheck(_motor); //This uses the "raw" values, and requires an extra function call
	if (_motor->Conv.Iu > g_hw_setup.Imax)
	{
		handleError(_motor, ERROR_OVERCURRENT_PHA);
	}
	if (_motor->Conv.Iv > g_hw_setup.Imax)
	{
		handleError(_motor, ERROR_OVERCURRENT_PHB);
	}
	if (_motor->Conv.Iw > g_hw_setup.Imax)
	{
		handleError(_motor, ERROR_OVERCURRENT_PHC);
	}
	if (_motor->Conv.Vbus > g_hw_setup.Vmax)
	{
		handleError(_motor, ERROR_OVERVOLTAGE);
	}
	if (_motor->Conv.Vbus < g_hw_setup.Vmin)
	{
		handleError(_motor, ERROR_UNDERVOLTAGE);
	}

// Deal with terrible hardware choice of only having two current sensors
// Based on Iu+Iv+Iw = 0
#ifdef MISSING_UCURRSENSOR
	_motor->Conv.Iu =
		-_motor->Conv.Iv - _motor->Conv.Iw;
#endif
#ifdef MISSING_VCURRSENSOR
	_motor->Conv.Iv =
		-_motor->Conv.Iu - _motor->Conv.Iw;
#endif
#ifdef MISSING_WCURRSENSOR
	_motor->Conv.Iw =
		-_motor->Conv.Iu - _motor->Conv.Iv;
#endif

#ifdef STEPPER_MOTOR // Skip the Clarke transform
	_motor->FOC.Iab.a = _motor->Conv.Iu;
	_motor->FOC.Iab.b = _motor->Conv.Iv;
#else

	// Power Variant Clark transform
	// Here we select the phases that have the lowest duty cycle to us, since
	// they should have the best current measurements
	switch (_motor->HighPhase)
	{
	case U:
		// Clark using phase V and W
		_motor->FOC.Iab.a = -_motor->Conv.Iv -
							_motor->Conv.Iw;
		_motor->FOC.Iab.b =
			CONST_1_R_SQRT_3_F * _motor->Conv.Iv -
			CONST_1_R_SQRT_3_F * _motor->Conv.Iw;
		break;
	case V:
		// Clark using phase U and W
		_motor->FOC.Iab.a = _motor->Conv.Iu;
		_motor->FOC.Iab.b =
			-CONST_1_R_SQRT_3_F * _motor->Conv.Iu -
			CONST_2_R_SQRT_3_F * _motor->Conv.Iw;
		break;
	case W:
		// Clark using phase U and V
		_motor->FOC.Iab.a = _motor->Conv.Iu;
		_motor->FOC.Iab.b =
			CONST_2_R_SQRT_3_F * _motor->Conv.Iv +
			CONST_1_R_SQRT_3_F * _motor->Conv.Iu;
		break;
	case N:
#ifdef USE_HIGHHOPES_PHASE_BALANCING
		_motor->FOC.Iab.g = 0.33f * (_motor->Conv.Iu + _motor->Conv.Iv + _motor->Conv.Iw);
		//		if(phasebalance){
		_motor->Conv.Iu = _motor->Conv.Iu - _motor->FOC.Iab.g;
		_motor->Conv.Iv = _motor->Conv.Iv - _motor->FOC.Iab.g;
		_motor->Conv.Iw = _motor->Conv.Iw - _motor->FOC.Iab.g;
		//		}
		if (fabs(_motor->FOC.Iab.g) > fabs(maxIgamma))
		{
			maxIgamma = _motor->FOC.Iab.g;
		}
		if (_motor->FOC.Vdq.q < 2.0f)
		{ // Reset it to reject accumulated random noise and enable multiple goes
			maxIgamma = 0.0f;
		}
#endif
		// Do the full transform
		_motor->FOC.Iab.a =
			0.66666f * _motor->Conv.Iu -
			0.33333f * _motor->Conv.Iv -
			0.33333f * _motor->Conv.Iw;
		_motor->FOC.Iab.b =
			CONST_1_R_SQRT_3_F * _motor->Conv.Iv -
			CONST_1_R_SQRT_3_F * _motor->Conv.Iw;
		break;
	} // End of phase selection switch
#endif
	// Park
	_motor->FOC.Idq.d = _motor->FOC.sincosangle.cos * _motor->FOC.Iab.a +
						_motor->FOC.sincosangle.sin * _motor->FOC.Iab.b;
	_motor->FOC.Idq.q = _motor->FOC.sincosangle.cos * _motor->FOC.Iab.b -
						_motor->FOC.sincosangle.sin * _motor->FOC.Iab.a;
}

void ADCPhaseConversion(MESC_motor_typedef *_motor)
{
	// To save clock cycles in the main run loop we only want to convert the phase voltages while tracking.
	// Convert the voltages to volts in real SI units
	_motor->Conv.Vu = (float)_motor->Raw.Vu * g_hw_setup.VBGain;
	_motor->Conv.Vv = (float)_motor->Raw.Vv * g_hw_setup.VBGain;
	_motor->Conv.Vw = (float)_motor->Raw.Vw * g_hw_setup.VBGain;
}

void MESC_Slow_IRQ_handler(MESC_motor_typedef *_motor)
{
	// #ifdef SLOWLED
	//	  SLOWLED->BSRR = SLOWLEDIO;
	// #endif
	slowLoop(_motor);
	// #ifdef SLOWLED
	//		SLOWLED->BSRR = SLOWLEDIO<<16U;
	// #endif
}
//extern uint32_t ADC_buffer[6];


void MESC_IC_Init(
#ifdef IC_TIMER
	TIM_HandleTypeDef _IC_TIMER
#endif
)
{
#ifdef IC_TIMER
	_IC_TIMER.Instance->SMCR = 84;
	_IC_TIMER.Instance->DIER = 3;
	_IC_TIMER.Instance->SR = 0;
	_IC_TIMER.Instance->CCMR1 = 513;
	_IC_TIMER.Instance->CCER = 49;
	_IC_TIMER.Instance->ARR = 65000;
	_IC_TIMER.Instance->DMAR = 1;
#ifdef IC_TIMER_RCPWM
	_IC_TIMER.Instance->PSC = (HAL_RCC_GetHCLKFreq() / (1000000 * SLOWTIM_SCALER)) - 1;
#else // RCtimer is used for PWM encoder
	_IC_TIMER.Instance->PSC = (HAL_RCC_GetHCLKFreq() / (4119000 * SLOWTIM_SCALER)) - 1;
	// The encoder PWM timers have a nominal frequency of 1kHz with 4119 levels

#endif
	IC_TIM_GPIO->MODER |= MODE_AF << (2 * IC_TIM_IONO);
	IC_TIM_GPIO->AFR[0] |= 0x2 << (IC_TIM_IONO * 4);
	//__HAL_TIM_ENABLE_IT(_IC_TIMER,TIM_IT_UPDATE);
	_IC_TIMER.Instance->CR1 = 5;
#endif
}

uint32_t SRtemp2, SRtemp3;
void MESC_IC_IRQ_Handler(MESC_motor_typedef *_motor, uint32_t SR, uint32_t CCR1, uint32_t CCR2)
{
#ifdef IC_TIMER_RCPWM
	if ((SR & 0x4) && !(SR & 0x1))
	{
		SRtemp2 = SR;
		input_vars.pulse_recieved = 1;
		input_vars.IC_duration = CCR1;
		input_vars.IC_pulse = CCR2;
	}
	if (SR & 0x1)
	{
		SRtemp3 = SR;
		input_vars.pulse_recieved = 0;
	}
#endif
#ifdef IC_TIMER_ENCODER // This will be for the encoder I guess...
	// The encoder PWM timers have a nominal frequency of 1kHz with 4119 levels
	if ((SR & 0x2) && !(SR & 0x1))
	{
		SRtemp2 = SR;
		_motor->FOC.encoder_duration = CCR1;
		_motor->FOC.encoder_pulse = CCR2;
		_motor->FOC.encoder_OK = 1;

		if (CCR2 < 14 || CCR1 < 3500 || CCR1 > 4500)
		{
			// Handle the error?
		}
		if (CCR2 < 16)
		{ // No error but need to stop it from underflowing the following math
			CCR2 = 16;
		}
		uint16_t temp_enc_ang;
		temp_enc_ang = _motor->FOC.enc_offset +
					   (uint16_t)(((65536 * (CCR2 - 16)) / (CCR1 - 24) * (uint32_t)_motor->m.pole_pairs) % 65536);
		// Set the angles used and zero the counter
		if (_motor->FOC.encoder_polarity_invert)
		{
			_motor->FOC.last_enc_period = _motor->FOC.enc_period_count;
			_motor->FOC.enc_period_count = 0;
			_motor->FOC.enc_angle = 65536 - temp_enc_ang;
		}
		else
		{
			_motor->FOC.last_enc_period = _motor->FOC.enc_period_count;
			_motor->FOC.enc_angle = temp_enc_ang;
			_motor->FOC.enc_period_count = 0;
		}

		// Calculate the deltas and steps
		_motor->FOC.enc_pwm_step = 0.8f * _motor->FOC.enc_pwm_step +
								   0.2f * (((int16_t)(_motor->FOC.enc_angle - _motor->FOC.last_enc_angle)) / (_motor->FOC.last_enc_period + 0.1f));
		_motor->FOC.last_enc_angle = _motor->FOC.enc_angle;
	}
	// For sensorless-PWM encoder combined mode
	// Calculate the sin and cos coefficients for future use in the flux observer
	sin_cos_fast((_motor->FOC.enc_angle), &_motor->FOC.encsin, &_motor->FOC.enccos);

	if (SR & 0x1 || _motor->FOC.encoder_pulse < 14 || _motor->FOC.encoder_pulse > (_motor->FOC.encoder_duration - 7))
	{
		SRtemp3 = SR;
		_motor->FOC.encoder_OK = 0;
	}
#endif
}
